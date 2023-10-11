#include "XPBD/XPBDPosConstraint.h"

XPBDPosConstraint::XPBDPosConstraint(XPBDBody* B1, XPBDBody* B2, Eigen::Vector3f R1, Eigen::Vector3f R2, float Length, float comp)
    : XPBDConstraint(B1, B2, comp),
    r1(R1), r2(R2), length(Length)
{
}
XPBDPosConstraint::~XPBDPosConstraint()
{
}
void XPBDPosConstraint::solveConstraint(float dt)
{
    dx = (b1->x + r1) - (b2->x + r2);
    float C = dx.norm() - length;
    if(C == 0.0f)
        return;
    Eigen::Vector3f n = dx.normalized();

    float w1 = 1.0f / b1->mass;
    float w2 = 1.0f / b2->mass;

    if(b1->bodyType == BodyType::Rigid && b2->bodyType == BodyType::Rigid)
    {
        ;
    }

    float h2 = dt * dt;
    float alpha = compliance / h2;
    float dlambda = (-C - alpha * lambda) / (w1 + w2 + alpha);
    lambda += dlambda;
    Eigen::Vector3f p = dlambda * n;
    
    b1->translatePos(p / b1->mass);
    b2->translatePos(-p / b2->mass);
}