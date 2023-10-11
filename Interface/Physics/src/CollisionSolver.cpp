#include "CollisionSolver.h"

#include "PD/SoftBody.h"
#include "PD/Simulation.h"
#include <limits>
// #include <algorithm>

CollisionSolver::CollisionSolver(Simulation *sim)
    : m_simulation(sim)
{
}

CollisionSolver::~CollisionSolver()
{
}

void CollisionSolver::solve(const VectorX &velocities_p, VectorX &contact, VectorX &friction)
{
    SoftBody* softbody = m_simulation->GetSoftBody();
    contact.setZero(velocities_p.size());
    friction.setZero(velocities_p.size());

    const SparseMatrix Minv = softbody->inv_mass_matrix();
    const SparseMatrix M = softbody->mass_matrix();
    const VectorX negative_momentum = -M * velocities_p;
    auto &contacts = m_simulation->getContacts();

    if(contacts.size() == 0)
    {
        return;
    }

    ScalarType min_res, rel_err;
    min_res = rel_err = std::numeric_limits<ScalarType>::infinity();
    VectorX best_friction = friction;

    int max_iters = 10;
    for (int i = 0; i < max_iters && rel_err > LARGER_EPSILON; ++i)
    {
        const VectorX old_friction = friction;
        const VectorX old_contact = contact;

        // Projection to contact normal
        const VectorX N = negative_momentum - old_friction;
        for (auto &c : contacts)
        {
            const int vertexIndex = c->vertexIndex;
            EigenVector3 N_alpha = projectionContact(c, N.block_vector(vertexIndex));
            contact.block_vector(vertexIndex) = N_alpha;
        }

        // Projection to friction
        const VectorX F = negative_momentum - contact;
        for (auto &c : contacts)
        {
            const int vertexIndex = c->vertexIndex;
            EigenVector3 f = projectionFriction(c, F.block_vector(vertexIndex), contact.block_vector(vertexIndex));
            friction.block_vector(vertexIndex) = f;
        }
        // Compute relative error
        const VectorX friction_diff = friction - old_friction;
        rel_err = ((friction_diff.transpose() * Minv * friction_diff) / (old_friction.transpose() * Minv * old_friction))(0);

        // Debug::PrintVectorX(old_friction, 3, "old_friction");

        // Debug::PrintVectorX(friction, 3, "friction");

        // printf("\n rel_err: %f\n", rel_err);

        // Compute residual
        VectorX V = velocities_p + Minv * contact + Minv * friction;
        ScalarType residual = 0.0f;
        for (auto &c : contacts)
        {
            const int vertexIndex = c->vertexIndex;
            const EigenVector3 V_block = V.block_vector(vertexIndex);
            residual += std::abs(V_block.dot(contact.block_vector(vertexIndex)));
        }
        // Check if residual is smaller
        if (residual < min_res)
        {
            min_res = residual;
            best_friction = friction;
        }
    }

    friction = best_friction;
    // Projection to contact normal
    const VectorX N = negative_momentum - friction;
    for (auto &c : contacts)
    {
        const int vertexIndex = c->vertexIndex;
        EigenVector3 N_alpha = projectionContact(c, N.block_vector(vertexIndex));
        contact.block_vector(vertexIndex) = N_alpha;
    }

    // Print result for Debug
    // Contact
    // Debug::PrintVectorX(contact, 3, "Contact");
    // // Friction impulse
    // Debug::PrintVectorX(friction, 3, "friction");
#ifndef NODEBUG
    using namespace std;
    ofstream logfile("log/CollisionSolver.log");
    logfile << "Contact impulse:\n";
    logfile << Eigen::Map<EigenMatrixXX>(contact.data(), 3, contact.size() / 3).transpose() << "\n";
    logfile << "Friction impulse:\n";
    logfile << Eigen::Map<EigenMatrixXX>(friction.data(), 3, friction.size() / 3).transpose() << "\n";

    for(Contact* c : contacts)
    {
        c->PrintInfo(logfile);
    }
#endif
    
}

EigenVector3 CollisionSolver::projectionContact(Contact *contact, const EigenVector3 &momentum)
{
    EigenVector3 lambda = contact->J1 * momentum;
    

    lambda(0) = std::max(0.0f, lambda(0));
    lambda(1) = lambda(2) = 0.0f;

    contact->lambda(0) = lambda(0);

    return contact->J1.transpose() * lambda;
}

EigenVector3 CollisionSolver::projectionFriction(Contact *contact, const EigenVector3 &momentum, const EigenVector3 contact_impulse)
{
    EigenVector3 lambda = contact->J1 * momentum;
    lambda(0) = 0.0f;
    ScalarType length = lambda.norm();
    ScalarType max_friction = contact->mu * contact_impulse.norm();

    if(length > max_friction)
    {
        lambda = lambda.normalized() * max_friction;
    }

    contact->lambda(1) = lambda(1);
    contact->lambda(2) = lambda(2);

    return contact->J1.transpose() * lambda;
}
