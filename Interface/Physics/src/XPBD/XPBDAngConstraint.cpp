#include "XPBD/XPBDAngConstraint.h"

#include "XPBD/XPBDUtil.h"

XPBDAngConstraint::XPBDAngConstraint(XPBDBody *B1, XPBDBody *B2, const Eigen::Vector3f& N, float Angle, float comp)
    : XPBDConstraint(B1, B2, comp)
{
    a = N;
    angle = Angle;
}

XPBDAngConstraint::~XPBDAngConstraint()
{
}


void XPBDAngConstraint::solveConstraint(float dt)
{
    Eigen::Quaternionf b1q = body1->getRotation();
    Eigen::Quaternionf b2q = body2->getRotation();
    Eigen::Vector3f a1 = b1q * a;
    Eigen::Vector3f a2 = b2q * a;

    Eigen::Vector3f d_q = a1.cross(a2);
    float C = d_q.norm() - angle;
    if(C <= 0.0f)
        return;
    
    Eigen::Vector3f n = d_q.normalized();
    Eigen::Vector3f n1 = b1q.inverse() * n;
    Eigen::Vector3f n2 = b2q.inverse() * n;
    Eigen::Matrix3f I1inv = body1->getInertiaLocal().inverse();
    Eigen::Matrix3f I2inv = body2->getInertiaLocal().inverse();
    
    float w1 = n1.transpose() * I1inv * n1;
    float w2 = n2.transpose() * I2inv * n2;

    float h2 = dt * dt;
    float alpha = compliance / h2;
    float dlambda = (-C - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;
    Eigen::Vector3f p = dlambda * n;

    Eigen::Vector3f I1invP = I1inv * p;
    Eigen::Vector3f I2invP = I2inv * p;

    Eigen::Quaternionf q1 = b1q + 0.5f * Eigen::Quaternionf(0.0f, I1invP[0], I1invP[1], I1invP[2]) * b1q;
    Eigen::Quaternionf q2 = b2q + -0.5f * Eigen::Quaternionf(0.0f, I2invP[0], I2invP[1], I2invP[2]) * b2q;

    body1->setRotation(q1);
    body2->setRotation(q2);
}
