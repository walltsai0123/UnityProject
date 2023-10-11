#include "Contact.h"

#include "PD/SoftBody.h"

#ifndef NODEBUG
std::ofstream logfile = std::ofstream("./log/Contact.log");
#endif

Contact::Contact()
{
}

Contact::Contact(const EigenVector3 &_p, const EigenVector3 &_n, float _phi, SoftBody *sb, int vindex)
    : p(_p), n(_n), pene(0), softbody(sb), vertexIndex(vindex), k(1e6f), b(1e5f), mu(0.4f)
{
    computeContactFrame();
    computeJacobian();

    PrintInfo(logfile);
}

Contact::~Contact()
{
}

void Contact::PrintInfo(std::ofstream &out)
{
#ifndef NODEBUG
    using namespace std;
    out << "Contact\n";
    out << "n: " << n.transpose() << "\n";
    out << "t1: " << t1.transpose() << "\n";
    out << "t2: " << t2.transpose() << "\n";
    out << "p: " << p.transpose() << "\n";
    out << "pene: " << pene << "\n";
    out << "v id:\n";
    out << vertexIndex <<  "\n";
    out << softbody->current_position(vertexIndex).transpose() << "\n";
    out << "Jacobian\n";
    out << J1 << "\n";
    out << "Jacobian Minv\n";
    out << J1Minv << "\n";
    out << "Lambda\n";
    out << lambda.transpose() << "\n";
    out << "global imp\n";
    out << lambda.transpose() * J1 << "\n";
    out << endl;
#endif
}

void Contact::computeContactFrame()
{
    EigenVector3 dir(1, 0, 0);
    t1 = dir - (dir.dot(n)) * n;
    if (t1.norm() < 1e-5f)
    {
        // Fail-safe: use axis-aligned direction (0,0,-1)
        t1 = -n.cross(Eigen::Vector3f(0, 0, -1));
    }
    t1.normalize();

    t2 = n.cross(t1);
    t2.normalize();
}


void Contact::computeJacobian()
{
    J0.setZero(3, 6);
    J1.setZero(3, 3);

    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 3);

    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = pene;

    J1.block<1, 3>(0, 0) = n.transpose();
    J1.block<1, 3>(1, 0) = t1.transpose();
    J1.block<1, 3>(2, 0) = t2.transpose();

    // Compute the J M^-1
    J1Minv = softbody->vertexMassInv(vertexIndex) * J1;

    // softbody->vertexContacts[vertexIndex].push_back(this);
}
