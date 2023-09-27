#define USE_STL_QUEUE_IMPLEMENTATION
#include "Simulation.h"
#include "SoftBody.h"
#include "CollisionSolver.h"

#include <omp.h>
#include <limits>

Simulation::Simulation()
    : // m_lbfgs_queue(nullptr),
      m_verbose_show_converge(false),
      // m_verbose_show_optimization_time(false),
      m_verbose_show_energy(false),
      m_verbose_show_factorization_warning(true),
      m_integration_method(IntegrationMethod::INTEGRATION_IMPLICIT_EULER),
      m_optimization_method(OptimizationMethod::OPTIMIZATION_METHOD_LBFGS),
      m_h(0.02), m_sub_stepping(1),
      m_enable_openmp(true),
      m_material_type(MaterialType::MATERIAL_TYPE_COROT),
      m_stiffness_stretch(80), m_stiffness_bending(20), m_stiffness_kappa(100),
      m_stiffness_auto_laplacian_stiffness(true), m_stiffness_laplacian(180.722f),
      m_gravity_constant(9.81f),
      m_damping_coefficient(0.001f),
      // m_restitution_coefficient(1), m_friction_coefficient(0.001f),
      m_iterations_per_frame(10),
      m_enable_line_search(true),
      m_ls_step_size(1), m_ls_alpha(0.03f), m_ls_beta(0.5f),
      m_lbfgs_H0_type(LBFGSH0Type::LBFGS_H0_LAPLACIAN), m_lbfgs_m(5)
{
    logfile.open("log/sim.log");
}

Simulation::~Simulation()
{
    logfile.flush();
    logfile.close();
}

const std::vector<Contact *> Simulation::getContacts() const
{
    std::vector<Contact *> result;
    result.resize(m_contacts.size());
    for(int i = 0; i < result.size(); ++i)
    {
        result[i] = m_contacts[i].get();
    }
    return result;
}

// const std::vector<std::unique_ptr<Contact>> &Simulation::getContacts() const
// {
//     return m_collisionDetect->getContacts();
// }

void Simulation::Reset()
{
    // m_collisionDetect.reset(new CollisionDetect(this));
    m_collisionSolver.reset(new CollisionSolver(this));

    m_y.resize(m_softbody->m_system_dimension);
    m_external_force.resize(m_softbody->m_system_dimension);

    setupConstraints();

    SetMaterialProperty();

    // m_selected_attachment_constraint = NULL;
    m_step_mode = false;

    // lbfgs
    m_lbfgs_restart_every_frame = true;
    m_lbfgs_need_update_H0 = true;

    // solver type
    m_solver_type = SOLVER_TYPE_DIRECT_LLT;
    m_iterative_solver_max_iteration = 10;

    // volume
    m_restshape_volume = getVolume(m_softbody->m_current_positions);
    m_current_volume = m_restshape_volume;

    // collision
    // m_collision_constraints.clear();

#ifdef OUTPUT_LS_ITERATIONS
    g_total_ls_iterations = 0;
    g_total_iterations = 0;
#endif
}

void Simulation::Update(float dt)
{
    // update external force
    calculateExternalForce();

    ScalarType old_h = m_h;
    m_h = dt;
    m_h = m_h / m_sub_stepping;

    m_last_descent_dir.resize(m_softbody->m_system_dimension);
    m_last_descent_dir.setZero();

    for (unsigned int substepping_i = 0; substepping_i != m_sub_stepping; substepping_i++)
    {
        // update inertia term
        computeConstantVectorsYandZ();

        // update
        integrateImplicitMethod();
        // switch (m_integration_method)
        // {
        // case INTEGRATION_QUASI_STATICS:
        // case INTEGRATION_IMPLICIT_EULER:
        // case INTEGRATION_IMPLICIT_BDF2:
        // case INTEGRATION_IMPLICIT_MIDPOINT:
        // case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        //     integrateImplicitMethod();
        //     break;
        // }

        // damping
        dampVelocity();
    }

    //// volume
    // m_current_volume = getVolume(m_softbody->m_current_positions);

    if (m_verbose_show_energy)
    {
        if (m_integration_method == INTEGRATION_QUASI_STATICS)
        {
            ScalarType W = evaluatePotentialEnergy(m_softbody->m_current_positions);
            std::cout << "Potential Energy = " << W << std::endl;
        }
        else
        {
            // output total energy;
            ScalarType K = evaluateKineticEnergy(m_softbody->m_current_velocities);
            ScalarType W = evaluatePotentialEnergy(m_softbody->m_current_positions);
            ScalarType K_plus_W = K + W;
            std::cout << "Kinetic Energy = " << K << std::endl;
            std::cout << "Potential Energy = " << W << std::endl;
            std::cout << "Total Energy = " << K_plus_W << std::endl;
        }
    }
    m_h = old_h;

#ifndef NODEBUG
    std::ofstream logger("log/simE.log");
    ScalarType energy = 0;
    VectorX gradient;
    VectorX x = m_softbody->current_positions();
    gradient.setZero(m_softbody->m_system_dimension);
    for(int i = 0; i < m_constraints.size(); ++i)
    {
        Constraint* c = m_constraints[i];
        ScalarType E = c->EvaluateEnergy(x);
        energy = E;
        c->EvaluateGradient(x, gradient);
        logger << "i: " << E << "\n";
    }
    logger << "Energy: " << energy << "\n";
    logger << "Gradient\n" 
            << Eigen::Map<EigenMatrixXX>(gradient.data(), 3, gradient.size() / 3).transpose() << "\n";

    ScalarType K = evaluateKineticEnergy(m_softbody->m_current_velocities);
    ScalarType W = evaluatePotentialEnergy(m_softbody->m_current_positions);
    ScalarType K_plus_W = K + W;
    logger << "Kinetic Energy = " << K << std::endl;
    logger << "Potential Energy = " << W << std::endl;
    logger << "Total Energy = " << K_plus_W << std::endl;
    logger << std::endl;
    logger.close();
#endif
}

void Simulation::SetMaterialProperty()
{
    SetMaterialProperty(m_constraints);
}

void Simulation::SetMaterialProperty(std::vector<Constraint *> &constraints)
{
    for (int i = 0; i < constraints.size(); ++i)
    {
        auto &c = constraints[i];
        auto vertexIndex = (m_softbody->m_tets[i])[0];
        auto meshIndex = m_softbody->vertexIndexToMeshIndex[vertexIndex].first;
        MaterialType mType = (MaterialType)m_softbody->meshes[meshIndex]->state->materialType;
        ScalarType mu = m_softbody->meshes[meshIndex]->state->mu;
        ScalarType lambda = m_softbody->meshes[meshIndex]->state->lambda;

        switch (c->Type())
        {
        case CONSTRAINT_TYPE_TET:
            c->SetMaterialProperty(mType, mu, lambda, m_stiffness_kappa, m_stiffness_laplacian);
            break;
        case CONSTRAINT_TYPE_SPRING:
            c->SetMaterialProperty(m_stiffness_stretch);
            break;
        case CONSTRAINT_TYPE_SPRING_BENDING:
            c->SetMaterialProperty(m_stiffness_bending);
            break;
        case CONSTRAINT_TYPE_ATTACHMENT:
            c->SetMaterialProperty(m_stiffness_attachment);
            break;
        }
    }
    SetReprefactorFlag();

#ifndef NODEBUG
    logfile << "Constraint material\n";
    for (int i = 0; i < constraints.size(); ++i)
    {
        auto &c = constraints[i];
        MaterialType type;
        ScalarType mu, lambda, kappa;
        c->GetMaterialProperty(type, mu, lambda, kappa);
        logfile << i << ": "
                << type << " "
                << mu << " "
                << lambda << " "
                << kappa << "\n";
    }
    logfile << std::flush;
#endif
}

void Simulation::clearConstraints()
{
    for (unsigned int i = 0; i < m_constraints.size(); ++i)
    {
        delete m_constraints[i];
    }
    m_constraints.clear();
}

void Simulation::setupConstraints()
{
    clearConstraints();

    // m_stiffness_high = 1e5;

    ScalarType total_volume = 0;
    std::vector<SparseMatrixTriplet> mass_triplets;
    std::vector<SparseMatrixTriplet> mass_1d_triplets;
    mass_triplets.clear();
    mass_1d_triplets.clear();

    VectorX &x = m_softbody->m_current_positions;

    std::vector<EigenVector4I> &tets = m_softbody->m_tets;
    logfile << "Constraints init\n";
    for (unsigned int i = 0; i < tets.size(); ++i)
    {
        const EigenVector4I &tet = tets[i];
        TetConstraint *c = new TetConstraint(tet[0], tet[1], tet[2], tet[3], x);
        m_constraints.push_back(c);

        logfile << i << ": "
                << tet[0] << " "
                << tet[1] << " "
                << tet[2] << " "
                << tet[3] << "\n";
    }
    logfile << std::endl;
}

void Simulation::dampVelocity()
{
    if (std::abs(m_damping_coefficient) < EPSILON)
        return;

    m_softbody->m_current_velocities *= 1 - m_damping_coefficient;
}

void Simulation::calculateExternalForce()
{
    m_external_force.resize(m_softbody->m_system_dimension);
    m_external_force.setZero();

    // gravity
    for (unsigned int i = 0; i < m_softbody->m_vertices_number; ++i)
    {
        m_external_force[3 * i + 1] += -m_gravity_constant;
    }

#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendSparseMatrix(m_softbody->m_mass_matrix, "M");
#endif
    m_external_force = m_softbody->m_mass_matrix * m_external_force;
}

void Simulation::integrateImplicitMethod()
{
    // take a initial guess
    VectorX x = m_y;
    // VectorX x = m_softbody->m_current_positions;

    // init method specific constants
    // for l-bfgs only
    if (m_lbfgs_restart_every_frame == true)
    {
        m_lbfgs_need_update_H0 = true;
    }
    EigenMatrixx3 x_nx3(x.size() / 3, 3);

    ScalarType total_time = 1e-5f;
    if (m_step_mode)
    {
#ifdef ENABLE_MATLAB_DEBUGGING
        ScalarType energy = evaluateEnergy(x);
        VectorX gradient;
        evaluateGradient(x, gradient);
        ScalarType gradient_norm = gradient.norm();
        g_debugger->SendData(x, energy, gradient_norm, 0, total_time);
#endif // ENABLE_MATLAB_DEBUGGING
    }

    collisionDetection();

    // TimerWrapper t_optimization;
    // t_optimization.Tic();
    // g_lbfgs_timer.Tic();
    // g_lbfgs_timer.Pause();
    //  while loop until converge or exceeds maximum iterations
    bool converge = false;
    m_ls_is_first_iteration = true;
    for (m_current_iteration = 0; !converge && m_current_iteration < m_iterations_per_frame; ++m_current_iteration)
    {
        // g_integration_timer.Tic();
        switch (m_optimization_method)
        {
        case OPTIMIZATION_METHOD_GRADIENT_DESCENT:
            converge = performGradientDescentOneIteration(x);
            break;
        case OPTIMIZATION_METHOD_NEWTON:
            converge = performNewtonsMethodOneIteration(x);
            break;
        case OPTIMIZATION_METHOD_LBFGS:
            converge = performLBFGSOneIteration(x);
            break;
        default:
            break;
        }
        m_ls_is_first_iteration = false;
        // g_integration_timer.Toc();

        if (m_verbose_show_converge)
        {
            if (converge && m_current_iteration != 0)
            {
                std::cout << "Optimization Converged in iteration #" << m_current_iteration << std::endl;
            }
        }

        if (m_step_mode)
        {
#ifdef ENABLE_MATLAB_DEBUGGING
            ScalarType energy = evaluateEnergy(x);
            VectorX gradient;
            evaluateGradient(x, gradient);
            ScalarType gradient_norm = gradient.norm();
            total_time += g_integration_timer.DurationInSeconds();
            g_debugger->SendData(x, energy, gradient_norm, m_current_iteration + 1, total_time);
#endif // ENABLE_MATLAB_DEBUGGING
        }
    }

    // t_optimization.Toc();
    // t_optimization.Report("Optimization", m_verbose_show_optimization_time);
    // g_lbfgs_timer.Resume();
    // g_lbfgs_timer.TocAndReport("L-BFGS overhead", m_verbose_show_converge, TIMER_OUTPUT_MILLISECONDS);

    // Solve collision
    collisionSolve(x);

    // update constants
    updatePosAndVel(x);
}

void Simulation::collisionDetection()
{
    // m_collisionDetect->detectCollision();
    // m_collisionDetect->computeContactJacobians();
    m_contacts.clear();

    for(int i = 0; i < m_softbody->m_vertices_number; ++i)
    {
        EigenVector3 vertexPos = m_softbody->current_position(i);
        if(vertexPos.y() < 0)
        {
            EigenVector3 p = vertexPos;
            p.y() = 0;
            EigenVector3 n(0.0f, 1.0f, 0.0f);
            float pene = vertexPos.y();

            m_contacts.push_back(std::make_unique<Contact>(p, n, pene, m_softbody, i));
        }
    }
}

void Simulation::collisionSolve(VectorX &x)
{
    const VectorX predicted_velocities = (x - m_softbody->current_positions()) / m_h;
    VectorX normal_imp, friction_imp;

    m_collisionSolver->solve(predicted_velocities, normal_imp, friction_imp);

    VectorX velocities = predicted_velocities + m_softbody->inv_mass_matrix() * (normal_imp + friction_imp);

    x = m_softbody->current_positions() + velocities * m_h;
}

bool Simulation::performGradientDescentOneIteration(VectorX &x)
{
    // evaluate gradient direction
    VectorX gradient;
    evaluateGradient(x, gradient);

#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendVector(gradient, "g");
#endif

    if (gradient.norm() < EPSILON)
        return true;

    // assign descent direction
    // VectorX descent_dir = -m_softbody->m_inv_mass_matrix*gradient;
    VectorX descent_dir = -gradient;

    // line search
    ScalarType step_size = lineSearch(x, gradient, descent_dir);

    // update x
    x = x + descent_dir * step_size;

    // report convergence
    if (step_size < EPSILON)
        return true;
    else
        return false;
}

bool Simulation::performNewtonsMethodOneIteration(VectorX &x)
{
    // TimerWrapper timer; timer.Tic();
    //  evaluate gradient direction
    VectorX gradient;
    evaluateGradient(x, gradient, true);
    // QSEvaluateGradient(x, gradient, m_ss->m_quasi_static);
#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendVector(gradient, "g");
#endif

    // timer.TocAndReport("evaluate gradient", m_verbose_show_converge);
    // timer.Tic();

    // evaluate hessian matrix
    SparseMatrix hessian_1;
    evaluateHessian(x, hessian_1);
    // SparseMatrix hessian_2;
    // evaluateHessianSmart(x, hessian_2);

    SparseMatrix &hessian = hessian_1;

#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendSparseMatrix(hessian_1, "H");
    // g_debugger->SendSparseMatrix(hessian_2, "H2");
#endif

    // timer.TocAndReport("evaluate hessian", m_verbose_show_converge);
    // timer.Tic();
    VectorX descent_dir;

    linearSolve(descent_dir, hessian, gradient);
    descent_dir = -descent_dir;

    // timer.TocAndReport("solve time", m_verbose_show_converge);
    // timer.Tic();

    // line search
    ScalarType step_size = lineSearch(x, gradient, descent_dir);
    // if (step_size < EPSILON)
    //{
    //	std::cout << "correct step size to 1" << std::endl;
    //	step_size = 1;
    // }
    //  update x
    x = x + descent_dir * step_size;

    // if (step_size < EPSILON)
    //{
    //	printVolumeTesting(x);
    // }

    // timer.TocAndReport("line search", m_verbose_show_converge);
    // timer.Toc();
    // std::cout << "newton: " << timer.Duration() << std::endl;

    if (-descent_dir.dot(gradient) < EPSILON_SQUARE)
        return true;
    else
        return false;
}

bool Simulation::performLBFGSOneIteration(VectorX &x)
{
    bool converged = false;
    ScalarType current_energy;
    VectorX gf_k;

    // set xk and gfk
    if (m_ls_is_first_iteration || !m_enable_line_search)
    {
        current_energy = evaluateEnergyAndGradient(x, gf_k);
    }
    else
    {
        current_energy = m_ls_prefetched_energy;
        gf_k = m_ls_prefetched_gradient;
    }
    // current_energy = evaluateEnergyAndGradient(x, gf_k);

    if (m_lbfgs_need_update_H0) // first iteration
    {
        // clear sk and yk and alpha_k
#ifdef USE_STL_QUEUE_IMPLEMENTATION
        // stl implementation
        m_lbfgs_y_queue.clear();
        m_lbfgs_s_queue.clear();
#else
        // my implementation
        delete m_lbfgs_queue;
        m_lbfgs_queue = new QueueLBFGS(x.size(), m_lbfgs_m);
#endif

        // decide H0 and it's factorization precomputation
        switch (m_lbfgs_H0_type)
        {
        case LBFGS_H0_LAPLACIAN:
            prefactorize();
            break;
        default:
            // prefactorize();
            break;
        }

        // g_lbfgs_timer.Resume();
        //  store them before wipeout
        m_lbfgs_last_x = x;
        m_lbfgs_last_gradient = gf_k;

        // g_lbfgs_timer.Pause();
        //  first iteration
        VectorX r;
        LBFGSKernelLinearSolve(r, gf_k, 1);
        // g_lbfgs_timer.Resume();

        // update
        VectorX p_k = -r;
        // g_lbfgs_timer.Pause();

        if (-p_k.dot(gf_k) < EPSILON_SQUARE || p_k.norm() / x.norm() < LARGER_EPSILON)
        {
            converged = true;
        }

        ScalarType alpha_k = linesearchWithPrefetchedEnergyAndGradientComputing(x, current_energy, gf_k, p_k, m_ls_prefetched_energy, m_ls_prefetched_gradient);
        x += alpha_k * p_k;

        // final touch
        m_lbfgs_need_update_H0 = false;
    }
    else // otherwise
    {
        // TimerWrapper t_local;
        // TimerWrapper t_global;
        // TimerWrapper t_linesearch;
        // TimerWrapper t_other;
        bool verbose = false;

        // t_other.Tic();
        // g_lbfgs_timer.Resume();
        //  enqueue stuff
        VectorX s_k = x - m_lbfgs_last_x;
        VectorX y_k = gf_k - m_lbfgs_last_gradient;

#ifdef USE_STL_QUEUE_IMPLEMENTATION
        // stl implementation
        if (m_lbfgs_s_queue.size() > m_lbfgs_m)
        {
            m_lbfgs_s_queue.pop_back();
            m_lbfgs_y_queue.pop_back();
        }
        // enqueue stuff
        m_lbfgs_s_queue.push_front(s_k);
        m_lbfgs_y_queue.push_front(y_k);

        int m_queue_size = m_lbfgs_s_queue.size();
#else
        // my implementation
        if (m_lbfgs_queue->isFull())
        {
            m_lbfgs_queue->dequeue();
        }
        m_lbfgs_queue->enqueue(s_k, y_k);

        int m_queue_size = m_lbfgs_queue->size();
#endif

        // store them before wipeout
        m_lbfgs_last_x = x;
        m_lbfgs_last_gradient = gf_k;
        VectorX q = gf_k;

        // loop 1 of l-BFGS
        std::vector<ScalarType> rho;
        rho.clear();
        std::vector<ScalarType> alpha;
        alpha.clear();
        int m_queue_visit_upper_bound = (m_lbfgs_m < m_queue_size) ? m_lbfgs_m : m_queue_size;
        ScalarType *s_i = NULL;
        ScalarType *y_i = NULL;
        for (int i = 0; i != m_queue_visit_upper_bound; i++)
        {
#ifdef USE_STL_QUEUE_IMPLEMENTATION
            // stl implementation
            ScalarType yi_dot_si = m_lbfgs_y_queue[i].dot(m_lbfgs_s_queue[i]);
            if (yi_dot_si < EPSILON_SQUARE)
            {
                return true;
            }
            ScalarType rho_i = 1.0 / yi_dot_si;
            rho.push_back(rho_i);
            alpha.push_back(rho[i] * m_lbfgs_s_queue[i].dot(q));
            q = q - alpha[i] * m_lbfgs_y_queue[i];
#else
            // my implementation
            m_lbfgs_queue->visitSandY(&s_i, &y_i, i);
            Eigen::Map<const VectorX> s_i_eigen(s_i, x.size());
            Eigen::Map<const VectorX> y_i_eigen(y_i, x.size());
            ScalarType yi_dot_si = (y_i_eigen.dot(s_i_eigen));
            if (yi_dot_si < EPSILON_SQUARE)
            {
                return true;
            }
            ScalarType rho_i = 1.0 / yi_dot_si;
            rho.push_back(rho_i);
            ScalarType alpha_i = rho_i * s_i_eigen.dot(q);
            alpha.push_back(alpha_i);
            q -= alpha_i * y_i_eigen;
#endif
        }
        // compute H0 * q
        // g_lbfgs_timer.Pause();
        // t_other.Pause();
        // t_global.Tic();
        VectorX r;
        // compute the scaling parameter on the fly
        ScalarType scaling_parameter = (s_k.transpose() * y_k).trace() / (y_k.transpose() * y_k).trace();
        if (scaling_parameter < EPSILON) // should not be negative
        {
            scaling_parameter = EPSILON;
        }
        LBFGSKernelLinearSolve(r, q, scaling_parameter);
        // t_global.Toc();
        // t_other.Resume();
        // g_lbfgs_timer.Resume();
        //  loop 2 of l-BFGS
        for (int i = m_queue_visit_upper_bound - 1; i >= 0; i--)
        {
#ifdef USE_STL_QUEUE_IMPLEMENTATION
            // stl implementation
            ScalarType beta = rho[i] * m_lbfgs_y_queue[i].dot(r);
            r = r + m_lbfgs_s_queue[i] * (alpha[i] - beta);
#else
            // my implementation
            m_lbfgs_queue->visitSandY(&s_i, &y_i, i);
            Eigen::Map<const VectorX> s_i_eigen(s_i, x.size());
            Eigen::Map<const VectorX> y_i_eigen(y_i, x.size());
            ScalarType beta = rho[i] * y_i_eigen.dot(r);
            r += s_i_eigen * (alpha[i] - beta);
#endif
        }
        // update
        VectorX p_k = -r;
        if (-p_k.dot(gf_k) < EPSILON_SQUARE || p_k.squaredNorm() < EPSILON_SQUARE)
        {
            converged = true;
        }
        // g_lbfgs_timer.Pause();
        // t_other.Toc();

        // t_linesearch.Tic();
        // ScalarType alpha_k = lineSearch(x, gf_k, p_k);
        ScalarType alpha_k = linesearchWithPrefetchedEnergyAndGradientComputing(x, current_energy, gf_k, p_k, m_ls_prefetched_energy, m_ls_prefetched_gradient);
        // t_linesearch.Toc();

        x += alpha_k * p_k;

        // t_global.Report("Forward Backward Substitution", verbose, TIMER_OUTPUT_MICROSECONDS);
        // t_other.Report("Two loop overhead", verbose, TIMER_OUTPUT_MICROSECONDS);
        // t_linesearch.Report("Linesearch", verbose, TIMER_OUTPUT_MICROSECONDS);
    }

    return converged;
}

void Simulation::LBFGSKernelLinearSolve(VectorX &r, VectorX rhs, ScalarType scaled_identity_constant)
{
    r.resize(rhs.size());
    switch (m_lbfgs_H0_type)
    {
    case LBFGS_H0_IDENTITY:
        r = rhs / scaled_identity_constant;
        break;
    case LBFGS_H0_LAPLACIAN: // h^2*laplacian+mass
    {
        // solve the linear system in reduced dimension because of the pattern of the Laplacian matrix
        // convert to nx3 space
        EigenMatrixx3 rhs_n3(rhs.size() / 3, 3);
        Vector3mx1ToMatrixmx3(rhs, rhs_n3);
        // solve using the nxn laplacian
        EigenMatrixx3 r_n3;
        if (m_solver_type == SOLVER_TYPE_CG)
        {
            m_preloaded_cg_solver_1D.setMaxIterations(m_iterative_solver_max_iteration);
            r_n3 = m_preloaded_cg_solver_1D.solve(rhs_n3);
        }
        else
        {
            r_n3 = m_prefactored_solver_1D.solve(rhs_n3);
        }
        // convert the result back
        Matrixmx3ToVector3mx1(r_n3, r);

        ////// conventional solve using 3nx3n system
        // if (m_solver_type == SOLVER_TYPE_CG)
        //{
        //	m_preloaded_cg_solver.setMaxIterations(m_iterative_solver_max_iteration);
        //	r = m_preloaded_cg_solver.solve(rhs);
        // }
        // else
        //{
        //	r = m_prefactored_solver.solve(rhs);
        // }
    }
    break;
    default:
        break;
    }
}

void Simulation::computeConstantVectorsYandZ()
{
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        m_y = m_softbody->m_current_positions;
        break;
    case INTEGRATION_IMPLICIT_EULER:
        m_y = m_softbody->m_current_positions + m_softbody->m_current_velocities * m_h + m_h * m_h * m_softbody->m_inv_mass_matrix * m_external_force;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        m_y = (4 * m_softbody->m_current_positions - m_softbody->m_previous_positions) / 3 + (4 * m_softbody->m_current_velocities - m_softbody->m_previous_velocities + 2 * m_h * m_softbody->m_inv_mass_matrix * m_external_force) * m_h * 2.0 / 9.0;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        m_y = m_softbody->m_current_positions + m_softbody->m_current_velocities * m_h + 0.5 * m_h * m_h * m_softbody->m_inv_mass_matrix * m_external_force;
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        m_y = m_softbody->m_current_positions + m_softbody->m_current_velocities * m_h + 0.5 * m_h * m_h * m_softbody->m_inv_mass_matrix * m_external_force;
        // evaluateGradientPureConstraint(m_softbody->m_current_positions, m_external_force, m_z);
        break;
    default:
        break;
    }
}

void Simulation::updatePosAndVel(const VectorX &new_pos)
{
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        m_softbody->m_previous_positions = m_softbody->m_current_positions;
        m_softbody->m_current_positions = new_pos;
        break;
    case INTEGRATION_IMPLICIT_EULER:
        m_softbody->m_previous_velocities = m_softbody->m_current_velocities;
        m_softbody->m_previous_positions = m_softbody->m_current_positions;
        m_softbody->m_current_velocities = (new_pos - m_softbody->m_current_positions) / m_h;
        m_softbody->m_current_positions = new_pos;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
    {
        m_softbody->m_previous_velocities = m_softbody->m_current_velocities;
        m_softbody->m_current_velocities = 1.5 * (new_pos - (4 * m_softbody->m_current_positions - m_softbody->m_previous_positions) / 3) / m_h;
        ;
        m_softbody->m_previous_positions = m_softbody->m_current_positions;
        m_softbody->m_current_positions = new_pos;
    }
    break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        m_softbody->m_previous_velocities = m_softbody->m_current_velocities;
        m_softbody->m_previous_positions = m_softbody->m_current_positions;
        m_softbody->m_current_velocities = 2 * (new_pos - m_softbody->m_current_positions) / m_h - m_softbody->m_current_velocities;
        m_softbody->m_current_positions = new_pos;
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        m_softbody->m_previous_velocities = m_softbody->m_current_velocities;
        m_softbody->m_previous_positions = m_softbody->m_current_positions;
        m_softbody->m_current_velocities = 2 * (new_pos - m_softbody->m_current_positions) / m_h - m_softbody->m_current_velocities;
        m_softbody->m_current_positions = new_pos;
        break;
    default:
        break;
    }
}

ScalarType Simulation::evaluateEnergy(const VectorX &x)
{
    ScalarType energy_pure_constraints, energy;

    ScalarType inertia_term = 0.5 * (x - m_y).transpose() * m_softbody->m_mass_matrix * (x - m_y);
    ScalarType h_square = m_h * m_h;
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        energy = evaluateEnergyPureConstraint(x, m_external_force);
        energy -= m_external_force.dot(x);
        break;
    case INTEGRATION_IMPLICIT_EULER:
        energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
        energy = inertia_term + h_square * energy_pure_constraints;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
        energy = inertia_term + h_square * 4.0 / 9.0 * energy_pure_constraints;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        energy_pure_constraints = evaluateEnergyPureConstraint((x + m_softbody->m_current_positions) / 2, m_external_force);
        energy = inertia_term + h_square * (energy_pure_constraints);
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        energy_pure_constraints = evaluateEnergyPureConstraint(x, m_external_force);
        energy = inertia_term + h_square / 4 * (energy_pure_constraints + m_z.dot(x));
        break;
    }

    return energy;
}

void Simulation::evaluateGradient(const VectorX &x, VectorX &gradient, bool enable_omp)
{
    ScalarType h_square = m_h * m_h;
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        evaluateGradientPureConstraint(x, m_external_force, gradient);
        gradient -= m_external_force;
        break; // DO NOTHING
    case INTEGRATION_IMPLICIT_EULER:
        evaluateGradientPureConstraint(x, m_external_force, gradient);
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square * gradient;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        evaluateGradientPureConstraint(x, m_external_force, gradient);
        gradient = m_softbody->m_mass_matrix * (x - m_y) + (h_square * 4.0 / 9.0) * gradient;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        evaluateGradientPureConstraint((x + m_softbody->m_current_positions) / 2, m_external_force, gradient);
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square / 2 * (gradient);
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        evaluateGradientPureConstraint(x, m_external_force, gradient);
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square / 4 * (gradient + m_z);
        break;
    }
}

ScalarType Simulation::evaluateEnergyAndGradient(const VectorX &x, VectorX &gradient)
{
    ScalarType h_square = m_h * m_h;
    ScalarType energy_pure_constraints, energy;
    ScalarType inertia_term = 0.5 * (x - m_y).transpose() * m_softbody->m_mass_matrix * (x - m_y);

    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        energy = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
        energy -= m_external_force.dot(x);
        gradient -= m_external_force;
        break; // DO NOTHING
    case INTEGRATION_IMPLICIT_EULER:
        energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
        energy = inertia_term + h_square * energy_pure_constraints;
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square * gradient;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
        energy = inertia_term + h_square * 4.0 / 9.0 * energy_pure_constraints;
        gradient = m_softbody->m_mass_matrix * (x - m_y) + (h_square * 4.0 / 9.0) * gradient;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        energy_pure_constraints = evaluateEnergyAndGradientPureConstraint((x + m_softbody->m_current_positions) / 2, m_external_force, gradient);
        energy = inertia_term + h_square * (energy_pure_constraints);
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square / 2 * (gradient);
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        energy_pure_constraints = evaluateEnergyAndGradientPureConstraint(x, m_external_force, gradient);
        energy = inertia_term + h_square / 4 * (energy_pure_constraints + m_z.dot(x));
        gradient = m_softbody->m_mass_matrix * (x - m_y) + h_square / 4 * (gradient + m_z);
        break;
    }

    return energy;
}

void Simulation::evaluateHessian(const VectorX &x, SparseMatrix &hessian_matrix)
{
    ScalarType h_square = m_h * m_h;
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        evaluateHessianPureConstraint(x, hessian_matrix);
        break; // DO NOTHING
    case INTEGRATION_IMPLICIT_EULER:
        evaluateHessianPureConstraint(x, hessian_matrix);
        hessian_matrix = m_softbody->m_mass_matrix + h_square * hessian_matrix;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        evaluateHessianPureConstraint(x, hessian_matrix);
        hessian_matrix = m_softbody->m_mass_matrix + h_square * 4.0 / 9.0 * hessian_matrix;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        evaluateHessianPureConstraint((x + m_softbody->m_current_positions) / 2, hessian_matrix);
        hessian_matrix = m_softbody->m_mass_matrix + h_square / 4 * hessian_matrix;
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        evaluateHessianPureConstraint(x, hessian_matrix);
        hessian_matrix = m_softbody->m_mass_matrix + h_square / 4 * hessian_matrix;
        break;
    }
}
void Simulation::evaluateLaplacian(SparseMatrix &laplacian_matrix)
{
    evaluateLaplacianPureConstraint(laplacian_matrix);

#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendSparseMatrix(m_weighted_laplacian, "L");
#endif

    ScalarType h_square = m_h * m_h;
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        break; // DO NOTHING
    case INTEGRATION_IMPLICIT_EULER:
        laplacian_matrix = m_softbody->m_mass_matrix + h_square * laplacian_matrix;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        laplacian_matrix = m_softbody->m_mass_matrix + h_square * 4.0 / 9.0 * laplacian_matrix;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        laplacian_matrix = m_softbody->m_mass_matrix + h_square / 4 * laplacian_matrix;
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        laplacian_matrix = m_softbody->m_mass_matrix + h_square / 4 * laplacian_matrix;
        break;
    }

#ifdef ENABLE_MATLAB_DEBUGGING
    g_debugger->SendSparseMatrix(laplacian_matrix, "A");
#endif
}

void Simulation::evaluateLaplacian1D(SparseMatrix &laplacian_matrix_1d)
{
    evaluateLaplacianPureConstraint1D(laplacian_matrix_1d);

    ScalarType h_square = m_h * m_h;
    switch (m_integration_method)
    {
    case INTEGRATION_QUASI_STATICS:
        break; // DO NOTHING
    case INTEGRATION_IMPLICIT_EULER:
        laplacian_matrix_1d = m_softbody->m_mass_matrix_1d + h_square * laplacian_matrix_1d;
        break;
    case INTEGRATION_IMPLICIT_BDF2:
        laplacian_matrix_1d = m_softbody->m_mass_matrix_1d + h_square * 4.0 / 9.0 * laplacian_matrix_1d;
        break;
    case INTEGRATION_IMPLICIT_MIDPOINT:
        laplacian_matrix_1d = m_softbody->m_mass_matrix_1d + h_square / 4 * laplacian_matrix_1d;
        break;
    case INTEGRATION_IMPLICIT_NEWMARK_BETA:
        laplacian_matrix_1d = m_softbody->m_mass_matrix_1d + h_square / 4 * laplacian_matrix_1d;
        break;
    }
}

ScalarType Simulation::evaluatePotentialEnergy(const VectorX &x)
{
    ScalarType energy = evaluateEnergyPureConstraint(x, m_external_force);
    energy -= m_external_force.dot(x);

    return energy;
}
ScalarType Simulation::evaluateKineticEnergy(const VectorX &v)
{
    return (0.5 * v.transpose() * m_softbody->m_mass_matrix * v);
}

ScalarType Simulation::evaluateEnergyPureConstraint(const VectorX &x, const VectorX &f_ext)
{
    ScalarType energy = 0.0;

    if (!m_enable_openmp)
    {
        for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            energy += (*it)->EvaluateEnergy(x);
        }
    }
    else
    {
        // openmp get all energy
        int i;
#pragma omp parallel
        {
#pragma omp for
            for (i = 0; i < m_constraints.size(); i++)
            {
                m_constraints[i]->EvaluateEnergy(x);
            }
        }

        // reduction
        for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            energy += (*it)->GetEnergy();
        }
    }

    // energy -= f_ext.dot(x);

    //// collision
    // for (unsigned int i = 1; i * 3 < x.size(); i++)
    //{
    //	EigenVector3 xi = x.block_vector(i);
    //	EigenVector3 n;
    //	ScalarType d;
    //	if (m_scene->StaticIntersectionTest(xi, n, d))
    //	{

    //	}
    //}

    // hardcoded collision plane
    /* if (m_processing_collision)
    {
        energy += evaluateEnergyCollision(x);
    } */

    return energy;
}

void Simulation::evaluateGradientPureConstraint(const VectorX &x, const VectorX &f_ext, VectorX &gradient)
{
    gradient.resize(m_softbody->m_system_dimension);
    gradient.setZero();

    if (!m_enable_openmp)
    {
        // constraints single thread
        for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            (*it)->EvaluateGradient(x, gradient);
        }
    }
    else
    {
        // constraints omp
        int i;
#pragma omp parallel
        {
#pragma omp for
            for (i = 0; i < m_constraints.size(); i++)
            {
                m_constraints[i]->EvaluateGradient(x);
            }
        }

        for (i = 0; i < m_constraints.size(); i++)
        {
            m_constraints[i]->GetGradient(gradient);
        }
    }

    // hardcoded collision plane
    /* if (m_processing_collision)
    {
        VectorX gc;

        evaluateGradientCollision(x, gc);

        gradient += gc;
    } */
}

ScalarType Simulation::evaluateEnergyAndGradientPureConstraint(const VectorX &x, const VectorX &f_ext, VectorX &gradient)
{
    ScalarType energy = 0.0;
    gradient.resize(m_softbody->m_system_dimension);
    gradient.setZero();

    if (!m_enable_openmp)
    {
        // constraints single thread
        for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
        {
            energy += (*it)->EvaluateEnergyAndGradient(x, gradient);
        }
    }
    else
    {
        // constraints omp
        int i;
#pragma omp parallel
        {
#pragma omp for
            for (i = 0; i < m_constraints.size(); i++)
            {
                m_constraints[i]->EvaluateEnergyAndGradient(x);
            }
        }

        // collect the results in a sequential way
        for (i = 0; i < m_constraints.size(); i++)
        {
            energy += m_constraints[i]->GetEnergyAndGradient(gradient);
        }
    }

    // hardcoded collision plane
    /* if (m_processing_collision)
    {
        VectorX gc;

        energy += evaluateEnergyAndGradientCollision(x, gc);

        gradient += gc;
    } */

    return energy;
}

void Simulation::evaluateHessianPureConstraint(const VectorX &x, SparseMatrix &hessian_matrix)
{
    hessian_matrix.resize(m_softbody->m_system_dimension, m_softbody->m_system_dimension);
    std::vector<SparseMatrixTriplet> h_triplets;
    h_triplets.clear();

    for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        (*it)->EvaluateHessian(x, h_triplets, m_definiteness_fix);
    }

    hessian_matrix.setFromTriplets(h_triplets.begin(), h_triplets.end());

    /* if (m_processing_collision)
    {
        SparseMatrix HC;
        evaluateHessianCollision(x, HC);
        hessian_matrix += HC;
    } */
}

void Simulation::evaluateLaplacianPureConstraint(SparseMatrix &laplacian_matrix)
{
    laplacian_matrix.resize(m_softbody->m_system_dimension, m_softbody->m_system_dimension);
    std::vector<SparseMatrixTriplet> l_triplets;
    l_triplets.clear();

    for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        (*it)->EvaluateWeightedLaplacian(l_triplets);
    }

    laplacian_matrix.setFromTriplets(l_triplets.begin(), l_triplets.end());
}

void Simulation::evaluateLaplacianPureConstraint1D(SparseMatrix &laplacian_matrix_1d)
{
    laplacian_matrix_1d.resize(m_softbody->m_vertices_number, m_softbody->m_vertices_number);
    std::vector<SparseMatrixTriplet> l_1d_triplets;
    l_1d_triplets.clear();

    for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        (*it)->EvaluateWeightedLaplacian1D(l_1d_triplets);
    }

    laplacian_matrix_1d.setFromTriplets(l_1d_triplets.begin(), l_1d_triplets.end());
}

ScalarType Simulation::lineSearch(const VectorX &x, const VectorX &gradient_dir, const VectorX &descent_dir)
{
    if (m_enable_line_search)
    {
        VectorX x_plus_tdx(m_softbody->m_system_dimension);
        ScalarType t = 1.0 / m_ls_beta;
        // ScalarType t = m_ls_step_size/m_ls_beta;
        ScalarType lhs, rhs;

        ScalarType currentObjectiveValue;
        try
        {
            currentObjectiveValue = evaluateEnergy(x);
        }
        catch (const std::exception &e)
        {
            std::cout << e.what() << std::endl;
        }
        do
        {
#ifdef OUTPUT_LS_ITERATIONS
            g_total_ls_iterations++;
#endif
            t *= m_ls_beta;
            x_plus_tdx = x + t * descent_dir;

            lhs = 1e15;
            rhs = 0;
            try
            {
                lhs = evaluateEnergy(x_plus_tdx);
            }
            catch (const std::exception &)
            {
                continue;
            }
            rhs = currentObjectiveValue + m_ls_alpha * t * (gradient_dir.transpose() * descent_dir)(0);
            if (lhs >= rhs)
            {
                continue; // keep looping
            }

            break; // exit looping

        } while (t > 1e-5);

        if (t < 1e-5)
        {
            t = 0.0;
        }
        m_ls_step_size = t;

        if (m_verbose_show_converge)
        {
            std::cout << "Linesearch Stepsize = " << t << std::endl;
            std::cout << "lhs (current energy) = " << lhs << std::endl;
            std::cout << "previous energy = " << currentObjectiveValue << std::endl;
            std::cout << "rhs (previous energy + alpha * t * gradient.dot(descet_dir)) = " << rhs << std::endl;
        }

#ifdef OUTPUT_LS_ITERATIONS
        g_total_iterations++;
        if (g_total_iterations % OUTPUT_LS_ITERATIONS_EVERY_N_FRAMES == 0)
        {
            std::cout << "Avg LS Iterations = " << g_total_ls_iterations / g_total_iterations << std::endl;
            g_total_ls_iterations = 0;
            g_total_iterations = 0;
        }
#endif
        return t;
    }
    else
    {
        return m_ls_step_size;
    }
}

ScalarType Simulation::linesearchWithPrefetchedEnergyAndGradientComputing(const VectorX &x, const ScalarType current_energy, const VectorX &gradient_dir, const VectorX &descent_dir, ScalarType &next_energy, VectorX &next_gradient_dir)
{
    if (m_enable_line_search)
    {
        VectorX x_plus_tdx(m_softbody->m_system_dimension);
        ScalarType t = 1.0 / m_ls_beta;
        ScalarType lhs, rhs;

        ScalarType currentObjectiveValue = current_energy;

        do
        {
#ifdef OUTPUT_LS_ITERATIONS
            g_total_ls_iterations++;
#endif

            t *= m_ls_beta;
            x_plus_tdx = x + t * descent_dir;

            lhs = 1e15;
            rhs = 0;
            try
            {
                lhs = evaluateEnergyAndGradient(x_plus_tdx, next_gradient_dir);
            }
            catch (const std::exception &)
            {
                continue;
            }
            rhs = currentObjectiveValue + m_ls_alpha * t * (gradient_dir.transpose() * descent_dir)(0);
            if (lhs >= rhs)
            {
                continue; // keep looping
            }

            next_energy = lhs;
            break; // exit looping

        } while (t > 1e-5);

        if (t < 1e-5)
        {
            t = 0.0;
            next_energy = current_energy;
            next_gradient_dir = gradient_dir;
        }
        m_ls_step_size = t;

        if (m_verbose_show_converge)
        {
            std::cout << "Linesearch Stepsize = " << t << std::endl;
            std::cout << "lhs (current energy) = " << lhs << std::endl;
            std::cout << "previous energy = " << currentObjectiveValue << std::endl;
            std::cout << "rhs (previous energy + alpha * t * gradient.dot(descet_dir)) = " << rhs << std::endl;
        }

#ifdef OUTPUT_LS_ITERATIONS
        g_total_iterations++;
        if (g_total_iterations % OUTPUT_LS_ITERATIONS_EVERY_N_FRAMES == 0)
        {
            std::cout << "Avg LS Iterations = " << g_total_ls_iterations / g_total_iterations << std::endl;
            g_total_ls_iterations = 0;
            g_total_iterations = 0;
        }
#endif

        return t;
    }
    else
    {
        return m_ls_step_size;
    }
}

void Simulation::precomputeLaplacianWeights()
{
    for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        if ((*it)->Type() == CONSTRAINT_TYPE_TET)
        {
            m_stiffness_laplacian = (*it)->ComputeLaplacianWeight();
        }
    }
    SetReprefactorFlag();
}

void Simulation::setWeightedLaplacianMatrix()
{
    evaluateLaplacian(m_weighted_laplacian);
}

void Simulation::setWeightedLaplacianMatrix1D()
{
    evaluateLaplacian1D(m_weighted_laplacian_1D);
}

void Simulation::prefactorize()
{
    if (m_prefactorization_flag == false)
    {
        // update laplacian coefficients
        if (m_stiffness_auto_laplacian_stiffness)
        {
            precomputeLaplacianWeights();
        }
        else
        {
            SetMaterialProperty();
        }

        // full space laplacian 3n x 3n
        setWeightedLaplacianMatrix();
        factorizeDirectSolverLLT(m_weighted_laplacian, m_prefactored_solver, "Our Method"); // prefactorization of laplacian
        m_preloaded_cg_solver.compute(m_weighted_laplacian);                                // load the cg solver

        // reduced dim space laplacian nxn
        setWeightedLaplacianMatrix1D();
        factorizeDirectSolverLLT(m_weighted_laplacian_1D, m_prefactored_solver_1D, "Our Method Reduced Space");
        m_preloaded_cg_solver_1D.compute(m_weighted_laplacian_1D);

#ifdef ENABLE_MATLAB_DEBUGGING
        g_debugger->SendSparseMatrix(m_weighted_laplacian, "L");
        g_debugger->SendSparseMatrix(m_weighted_laplacian_1D, "L1");
#endif
        m_prefactorization_flag = true;
    }
}

ScalarType Simulation::getVolume(const VectorX &x)
{
    ScalarType volume = 0.0;
    for (std::vector<Constraint *>::iterator it = m_constraints.begin(); it != m_constraints.end(); ++it)
    {
        volume += (*it)->GetVolume(x);
    }

    return volume;
}

ScalarType Simulation::linearSolve(VectorX &x, const SparseMatrix &A, const VectorX &b, char *msg)
{
    ScalarType residual = 0;

    switch (m_solver_type)
    {
    case SOLVER_TYPE_DIRECT_LLT:
    {
#ifdef PARDISO_SUPPORT
        Eigen::PardisoLLT<SparseMatrix, Eigen::Upper> A_solver;
#else
        Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> A_solver;
#endif
        factorizeDirectSolverLLT(A, A_solver, msg);
        x = A_solver.solve(b);
    }
    break;
    case SOLVER_TYPE_CG:
    {
        x.resize(b.size());
        x.setZero();
        residual = conjugateGradientWithInitialGuess(x, A, b, m_iterative_solver_max_iteration);
    }
    break;
    default:
        break;
    }

    return residual;
}

ScalarType Simulation::conjugateGradientWithInitialGuess(VectorX &x, const SparseMatrix &A, const VectorX &b, const unsigned int max_it, const ScalarType tol)
{
    VectorX r = b - A * x;
    VectorX p = r;
    ScalarType rsold = r.dot(r);
    ScalarType rsnew;

    VectorX Ap;
    Ap.resize(x.size());
    ScalarType alpha;

    for (unsigned int i = 1; i != max_it; ++i)
    {
        Ap = A * p;
        alpha = rsold / p.dot(Ap);
        x = x + alpha * p;

        r = r - alpha * Ap;
        rsnew = r.dot(r);
        if (sqrt(rsnew) < tol)
        {
            break;
        }
        p = r + (rsnew / rsold) * p;
        rsold = rsnew;
    }

    return sqrt(rsnew);
}

void Simulation::factorizeDirectSolverLLT(const SparseMatrix &A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> &lltSolver, char *warning_msg)
{
    SparseMatrix A_prime = A;
    lltSolver.analyzePattern(A_prime);
    lltSolver.factorize(A_prime);
    ScalarType Regularization = 1e-10;
    bool success = true;
    SparseMatrix I;
    while (lltSolver.info() != Eigen::Success)
    {
        if (success == true) // first time factorization failed
        {
            EigenMakeSparseIdentityMatrix(A.rows(), A.cols(), I);
        }
        Regularization *= 10;
        A_prime = A_prime + Regularization * I;
        lltSolver.factorize(A_prime);
        success = false;
    }
    if (!success && m_verbose_show_factorization_warning)
        std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(llt solver)" << std::endl;
}
