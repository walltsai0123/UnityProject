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

void XPBDRigidBody::collectCollsion(float dt)
{
    updateAABB();
    float expandDist = 2 * dt * v.norm();

    Eigen::Vector3f planeN(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f planePoint(0,0,0);

    collidePlane = false;
    if(aabb.min.y() - expandDist < 0.0f && aabb.max.y() + expandDist > 0.0f)
        collidePlane = true;
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
    // omega = 2.0f * dq.vec() / dt;
    // omega = dq.w() >= 0 ? omega : -omega;

    Eigen::AngleAxisf aa(dq);
    omega = aa.angle() * aa.axis() / dt;
}

void XPBDRigidBody::velocitySolve(float dt)
{

}

void XPBDRigidBody::endFrame()
{
}

void XPBDRigidBody::updateAABB()
{
    if(geometry->getType() == eGeometryType::kSphere)
    {
        Sphere *sphere = dynamic_cast<Sphere*>(geometry.get());
        float radius = sphere->radius;
        aabb.min = x - Eigen::Vector3f(1,1,1) * radius;
        aabb.max = x + Eigen::Vector3f(1,1,1) * radius;
    }
    else if(geometry->getType() == eGeometryType::kBox)
    {
        Box *box = dynamic_cast<Box*>(geometry.get());
        float radius = (0.5f * box->dim).norm();
        aabb.min = x - Eigen::Vector3f(1,1,1) * radius;
        aabb.max = x + Eigen::Vector3f(1,1,1) * radius;
    }
    else if(geometry->getType() == eGeometryType::kCylinder)
    {
        Cylinder *cylinder = dynamic_cast<Cylinder*>(geometry.get());
        float radius = cylinder->radius;
        float height_half = cylinder->height / 2.0f;
        // Eigen::Vector3f axis = q * cylinder->axis;

        float sphereRadius = sqrtf(radius * radius + height_half * height_half);
        aabb.min = x - Eigen::Vector3f(1,1,1) * sphereRadius;
        aabb.max = x + Eigen::Vector3f(1,1,1) * sphereRadius;
    }
    
}

void XPBDRigidBody::updateInertia()
{
    I = q * Ibody * q.inverse();
    Iinv = q * IbodyInv * q.inverse();
}

void XPBDRigidBody::solveCollision(float dt)
{
    if(!collidePlane)
        return;
    
    Eigen::Vector3f planeN(0,1,0);
    Eigen::Vector3f contactPoint;
    
    if(geometry->getType() == eGeometryType::kCylinder)
    {
        Cylinder *cylinder = dynamic_cast<Cylinder*>(geometry.get());
        float radius = cylinder->radius;
        float height_half = cylinder->height / 2.0f;
        Eigen::Vector3f axis = q * cylinder->axis;

        float coef = (0.0f - x.y()) / axis.y();
        if(coef > height_half){

        }
        else{
        }
    }
}