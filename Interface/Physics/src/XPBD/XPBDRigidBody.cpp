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
    Eigen::Vector3f planePoint(0, 0, 0);

    collidePlane = false;
    if (aabb.min.y() - expandDist < 0.0f && aabb.max.y() + expandDist > 0.0f)
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
    float restitutionCoef;
    for(Collision &c : collisions)
    {
        float vn = c.surfaceN.dot(v);
        Eigen::Vector3f vt = v - vn * c.surfaceN;

        // tangent
        Eigen::Vector3f dvt = -vt.normalized() * std::min(dt * c.frictionCoef * c.fn.norm(), vt.norm());
        v += dvt;

        // normal
        restitutionCoef = (std::abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
        Eigen::Vector3f dvn = c.surfaceN * (-vn + std::min(0.0f, -restitutionCoef * c.vn_));
        x += dvn;
    }
}

void XPBDRigidBody::endFrame()
{
}

void XPBDRigidBody::updateAABB()
{
    if (geometry->getType() == eGeometryType::kSphere)
    {
        Sphere *sphere = dynamic_cast<Sphere *>(geometry.get());
        float radius = sphere->radius;
        aabb.min = x - Eigen::Vector3f(1, 1, 1) * radius;
        aabb.max = x + Eigen::Vector3f(1, 1, 1) * radius;
    }
    else if (geometry->getType() == eGeometryType::kBox)
    {
        Box *box = dynamic_cast<Box *>(geometry.get());
        float radius = (0.5f * box->dim).norm();
        aabb.min = x - Eigen::Vector3f(1, 1, 1) * radius;
        aabb.max = x + Eigen::Vector3f(1, 1, 1) * radius;
    }
    else if (geometry->getType() == eGeometryType::kCylinder)
    {
        Cylinder *cylinder = dynamic_cast<Cylinder *>(geometry.get());
        float radius = cylinder->radius;
        float height_half = cylinder->height / 2.0f;
        // Eigen::Vector3f axis = q * cylinder->axis;

        float sphereRadius = sqrtf(radius * radius + height_half * height_half);
        aabb.min = x - Eigen::Vector3f(1, 1, 1) * sphereRadius;
        aabb.max = x + Eigen::Vector3f(1, 1, 1) * sphereRadius;
    }
}

void XPBDRigidBody::updateInertia()
{
    I = q * Ibody * q.inverse();
    Iinv = q * IbodyInv * q.inverse();
}

void XPBDRigidBody::solveCollision(float dt)
{
    collisions.clear();
    if (!collidePlane)
        return;

    Eigen::Vector3f planeN(0, 1, 0);
    Eigen::Vector3f planePoint(0, 0, 0);
    computeContact(planeN, planePoint);


    for(Collision &C : collisions)
    {
        Eigen::Vector3f p1 = x + q * C.r;
        Eigen::Vector3f p2(0, 0, 0);
        float penetration = (p1 - p2).dot(C.surfaceN);
        if(penetration >= 0.0f)
            continue;

        Eigen::Vector3f dx = penetration * C.surfaceN;

        float w1 = 1.0f / mass;
        float alpha = 0.0f;
        float dlambda_n = -penetration / (w1 + alpha);
        Eigen::Vector3f p = dlambda_n * C.surfaceN;
        x += p * w1;

        Eigen::Vector3f dp = x - xprev;
        Eigen::Vector3f dpt = dp - dp.dot(C.surfaceN) * C.surfaceN;
        float C2 = dpt.norm();
        if(C2 < 1e-6)
            continue;

        float dlambta_t = -C2 / (w1 + alpha);
        Eigen::Vector3f pt = dlambta_t * dpt.normalized();
        x += pt * w1;
    }
    
}

void XPBDRigidBody::computeContact(const Eigen::Vector3f& planeNormal, const Eigen::Vector3f& planePoint)
{
    Eigen::Vector3f localPlaneN = q.conjugate() * planeNormal;
    Eigen::Vector3f localPlanePoint = q.conjugate() * (planePoint - x);

    if (geometry->getType() == eGeometryType::kCylinder)
    {
        Cylinder *cylinder = dynamic_cast<Cylinder *>(geometry.get());
        Eigen::Vector3f endPoints[2];
        endPoints[0] = 0.5f * cylinder->height * cylinder->axis;
        endPoints[1] = -0.5f * cylinder->height * cylinder->axis;
        
        Eigen::Vector3f dir = localPlaneN.cross(cylinder->axis).cross(cylinder->axis);

        Eigen::Vector3f corners[4] = 
        {
            endPoints[0] + cylinder->radius * dir,
            endPoints[0] - cylinder->radius * dir,
            endPoints[1] + cylinder->radius * dir,
            endPoints[1] - cylinder->radius * dir
        };
        
        for(Eigen::Vector3f& c : corners)
        {
            float penetration = (c - localPlanePoint).dot(localPlaneN);
            if(penetration < 0.0f)
            {
                Collision newCollision;
                newCollision.r = c;
                newCollision.surfaceN = planeNormal;
                newCollision.pene = penetration;

                collisions.push_back(newCollision);
            }
        }
    }
}