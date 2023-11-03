#include "XPBD/XPBDConstraint.h"

#include "XPBD/XPBDUtil.h"

int XPBDConstraint::counter = 0;

XPBDConstraint::XPBDConstraint(XPBDBody *B1, XPBDBody *B2, float comp) : b1(B1), b2(B2), compliance(comp), lambda(0.0f)
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

void XPBDConstraint::solvePosConstraint(float dt, const Eigen::Vector3f r1, const Eigen::Vector3f r2, float dmax, float comp)
{
    Eigen::Vector3f b1x = b1->getPosition();
    Eigen::Vector3f b2x = b2->getPosition();
    Eigen::Quaternionf b1q = b1->getRotation();
    Eigen::Quaternionf b2q = b2->getRotation();
    Eigen::Vector3f R1 = b1q * r1;
    Eigen::Vector3f R2 = b2q * r2;

    Eigen::Vector3f dr = (b1x + R1) - (b2x + R2);
    float C = dr.norm() - dmax;
    if (C <= 1e-6f)
        return;
    
    Eigen::Vector3f n = dr.normalized();
    Eigen::Vector3f n1 = b1q.inverse() * n;
    Eigen::Vector3f n2 = b2q.inverse() * n;
    Eigen::Matrix3f I1inv = b1->getInertiaLocal().inverse();
    Eigen::Matrix3f I2inv = b2->getInertiaLocal().inverse();

    Eigen::Vector3f r1xn = r1.cross(n1);
    Eigen::Vector3f r2xn = r2.cross(n2);

    float w1 = 1.0f / b1->getMass() + r1xn.transpose() * I1inv * r1xn;
    float w2 = 1.0f / b2->getMass() + r2xn.transpose() * I2inv * r2xn;

    float h2 = dt * dt;
    float alpha = comp / h2;
    float dlambda = (-C - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;
    Eigen::Vector3f p = dlambda * n;

    Eigen::Vector3f b1newX = b1x + p / b1->getMass();
    Eigen::Vector3f b2newX = b2x - p / b2->getMass();
    b1->setPosition(b1newX);
    b2->setPosition(b2newX);

    Eigen::Vector3f p1 = b1q.inverse() * p;
    Eigen::Vector3f p2 = b2q.inverse() * p;
    Eigen::Vector3f r1xp = I1inv * r1.cross(p1);
    Eigen::Quaternionf b1newRot = b1q + 0.5f * Eigen::Quaternionf(0.0f, r1xp(0), r1xp(1), r1xp(2)) * b1q;
    Eigen::Vector3f r2xp = I2inv * r2.cross(p2);
    Eigen::Quaternionf b2newRot = b2q + -0.5f * Eigen::Quaternionf(0.0f, r2xp(0), r2xp(1), r2xp(2)) * b2q;
    b1->setRotation(b1newRot.normalized());
    b2->setRotation(b2newRot.normalized());
}
void XPBDConstraint::solveAngConstraint(float dt, const Eigen::Vector3f dq, float angle, float comp)
{
    float C = dq.norm() - angle;
    if(C <= 1e-6f)
        return;

    Eigen::Quaternionf b1q = b1->getRotation();
    Eigen::Quaternionf b2q = b2->getRotation();   
    Eigen::Vector3f n = dq.normalized();
    Eigen::Vector3f n1 = b1q.inverse() * n;
    Eigen::Vector3f n2 = b2q.inverse() * n;

    Eigen::Matrix3f I1inv = b1->getInertiaLocal().inverse();
    Eigen::Matrix3f I2inv = b2->getInertiaLocal().inverse();
    
    float w1 = n1.transpose() * I1inv * n1;
    float w2 = n2.transpose() * I2inv * n2;

    float h2 = dt * dt;
    float alpha = comp / h2;
    float dlambda = (-C - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;
    Eigen::Vector3f p = dlambda * n;

    Eigen::Vector3f p1 = b1q.inverse() * p;
    Eigen::Vector3f p2 = b2q.inverse() * p;
    Eigen::Vector3f I1invP = I1inv * p1;
    Eigen::Vector3f I2invP = I2inv * p2;

    Eigen::Quaternionf q1 = b1q + 0.5f * Eigen::Quaternionf(0.0f, I1invP[0], I1invP[1], I1invP[2]) * b1q;
    Eigen::Quaternionf q2 = b2q + -0.5f * Eigen::Quaternionf(0.0f, I2invP[0], I2invP[1], I2invP[2]) * b2q;

    b1->setRotation(q1.normalized());
    b2->setRotation(q2.normalized());
}