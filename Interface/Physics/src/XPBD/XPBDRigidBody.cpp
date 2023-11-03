#include "XPBD/XPBDRigidBody.h"

#include "XPBD/XPBDUtil.h"

int XPBDRigidBody::counter = 0;

XPBDRigidBody::XPBDRigidBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, Eigen::Vector3f inertia, float Mass)
    : XPBDBody(BodyType::Rigid),
      v(0, 0, 0),
      omega(0, 0, 0),
      IbodyInv(Eigen::Matrix3f::Identity()),
      Iinv(Eigen::Matrix3f::Zero()),
      f(0, 0, 0),
      tau(0, 0, 0)
{
    std::string logfileName = "log/XPBDRigidBody_" + std::to_string(counter) + ".log";
    logfile.open(logfileName);

    mass = Mass;
    x = pos;
    q = rot;
    Ibody = inertia.asDiagonal();
    IbodyInv = Ibody.inverse();

    counter++;

#ifndef NODEBUG
    logfile << "Init\n";
    logfile << "Pos " << x.transpose() << "\n";
    logfile << "Rot " << q << "\n";
    logfile << "Mass " << mass << "\n";
    logfile << "Ibody\n"
            << Ibody << "\n";
    logfile << "IbodyInv\n"
            << IbodyInv << "\n";
    logfile.flush();
#endif
}

XPBDRigidBody::XPBDRigidBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, Geometry *_geometry, float Mass)
    : XPBDBody(BodyType::Rigid),
      v(0, 0, 0),
      omega(0, 0, 0),
      IbodyInv(Eigen::Matrix3f::Identity()),
      Iinv(Eigen::Matrix3f::Zero()),
      f(0, 0, 0),
      tau(0, 0, 0),
      geometry(_geometry)
{
    std::string logfileName = "log/XPBDRigidBody_" + std::to_string(counter) + ".log";
    logfile.open(logfileName);

    mass = Mass;
    x = pos;
    q = rot;
    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();

    counter++;

#ifndef NODEBUG
    logfile << "Init\n";
    logfile << "Pos " << x.transpose() << "\n";
    logfile << "Rot " << q << "\n";
    logfile << "Mass" << mass << "\n";
    logfile << "Geometry: " << geometry->toString() << "\n";
    logfile << "Ibody\n"
            << Ibody << "\n";
    logfile << "IbodyInv\n"
            << IbodyInv << "\n";
    logfile.flush();
#endif
}


XPBDRigidBody::~XPBDRigidBody()
{
    counter--;
    logfile.flush();
    logfile.close();
}
void XPBDRigidBody::preSolve(float dt)
{
    updateInertia();

    f = mass * Eigen::Vector3f(0.f, -9.81f, 0.f);
    tau.setZero();

    xprev = x;
    v += dt * (1.0f / mass) * f;
    x += dt * v;

    qprev = q;
    omega += dt * Iinv * (tau - omega.cross(I * omega));
    q = q + 0.5f * dt * Eigen::Quaternionf(0, omega[0], omega[1], omega[2]) * q;
    q.normalize();
}

void XPBDRigidBody::solve(float dt)
{
}

void XPBDRigidBody::postSolve(float dt)
{
    v = (x - xprev) / dt;
    Eigen::Quaternionf dq = q * qprev.inverse();
    // omega = 2.0f * Eigen::Vector3f(dq.x(), dq.y(), dq.z()) / dt;
    // omega = dq.w() >= 0 ? omega : -omega;

    Eigen::AngleAxisf aa(dq);
    omega = aa.angle() * aa.axis() / dt;
}

void XPBDRigidBody::endFrame()
{
}

void XPBDRigidBody::updateInertia()
{
    I = q * Ibody * q.inverse();
    Iinv = q * IbodyInv * q.inverse();
}
