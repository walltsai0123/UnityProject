#include "XPBD/XPBDConstraint.h"

#include "XPBD/XPBDUtil.h"

int XPBDConstraint::counter = 0;

XPBDConstraint::XPBDConstraint(XPBDBody *B1, XPBDBody *B2, float comp) : body1(B1), body2(B2), compliance(comp), lambda(0.0f)
{
    std::string logfileName = "log/XPBDConstraint_" + std::to_string(counter) + ".log";
    logfile.open(logfileName);
    counter++;
}


XPBDConstraint::~XPBDConstraint()
{
    counter--;
    logfile.flush();
    logfile.close();
}

void XPBDConstraint::solvePosConstraint(float dt, XPBDBody* body1, XPBDBody* body2, const Eigen::Vector3f &r1, const Eigen::Vector3f &r2, 
    const Eigen::Vector3f &dx, float dmax, float comp)
{
    if(body1 == nullptr && body2 == nullptr)
        return;
    
    float C = dx.norm() - dmax;
    if (C <= 1e-6f)
        return;

    Eigen::Vector3f b1x = body1->getPosition();
    Eigen::Vector3f b2x = body2->getPosition();
    Eigen::Quaternionf b1q = body1->getRotation();
    Eigen::Quaternionf b2q = body2->getRotation();
    Eigen::Vector3f R1 = b1q * r1;
    Eigen::Vector3f R2 = b2q * r2;

    Eigen::Vector3f n = dx.normalized();
    Eigen::Matrix3f I1inv = b1q * body1->getInertiaLocal().inverse() * b1q.inverse();
    Eigen::Matrix3f I2inv = b2q * body2->getInertiaLocal().inverse() * b2q.inverse();

    Eigen::Vector3f r1xn = R1.cross(n);
    Eigen::Vector3f r2xn = R2.cross(n);

    float w1 = 1.0f / body1->getMass() + r1xn.transpose() * I1inv * r1xn;
    float w2 = 1.0f / body2->getMass() + r2xn.transpose() * I2inv * r2xn;

    float h2 = dt * dt;
    float alpha = comp / h2;
    float dlambda = (-C - alpha) / (w1 + w2 + alpha);
    Eigen::Vector3f p = dlambda * n;

    Eigen::Vector3f b1newX = b1x + p / body1->getMass();
    Eigen::Vector3f b2newX = b2x - p / body2->getMass();
    body1->setPosition(b1newX);
    body2->setPosition(b2newX);

    Eigen::Vector3f r1xp = I1inv * r1.cross(p);
    Eigen::Quaternionf b1newRot = b1q + 0.5f * Eigen::Quaternionf(0.0f, r1xp(0), r1xp(1), r1xp(2)) * b1q;
    Eigen::Vector3f r2xp = I2inv * r2.cross(p);
    Eigen::Quaternionf b2newRot = b2q + -0.5f * Eigen::Quaternionf(0.0f, r2xp(0), r2xp(1), r2xp(2)) * b2q;
    body1->setRotation(b1newRot.normalized());
    body2->setRotation(b2newRot.normalized());
}
void XPBDConstraint::solveAngConstraint(float dt, XPBDBody* body1, XPBDBody* body2, const Eigen::Vector3f &dq, float angle, float comp)
{
    if(body1 == nullptr && body2 == nullptr)
        return;
    
    float C = dq.norm() - angle;
    if(std::abs(C) <= 1e-6f)
        return;

    Eigen::Quaternionf b1q = body1->getRotation();
    Eigen::Quaternionf b2q = body2->getRotation();   
    Eigen::Vector3f n = dq.normalized();

    Eigen::Matrix3f I1inv = b1q * body1->getInertiaLocal().inverse() * b1q.inverse();
    Eigen::Matrix3f I2inv = b2q * body2->getInertiaLocal().inverse() * b2q.inverse();
    
    float w1 = n.transpose() * I1inv * n;
    float w2 = n.transpose() * I2inv * n;

    float h2 = dt * dt;
    float alpha = comp / h2;
    float dlambda = (-C - alpha) / (w1 + w2 + alpha);
    Eigen::Vector3f p = dlambda * n;

    Eigen::Vector3f I1invP = I1inv * p;
    Eigen::Vector3f I2invP = I2inv * p;

    Eigen::Quaternionf q1 = b1q + 0.5f * Eigen::Quaternionf(0.0f, I1invP[0], I1invP[1], I1invP[2]) * b1q;
    Eigen::Quaternionf q2 = b2q + -0.5f * Eigen::Quaternionf(0.0f, I2invP[0], I2invP[1], I2invP[2]) * b2q;

    body1->setRotation(q1.normalized());
    body2->setRotation(q2.normalized());
}