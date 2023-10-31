#include "XPBD/XPBDAttatchment.h"

#include "XPBD/XPBDRigidBody.h"
#include "XPBD/XPBDSoftBody.h"

XPBDAttatchment::XPBDAttatchment(XPBDBody *b1, XPBDBody *b2)
    : XPBDConstraint(b1, b2, 0.0f)
{
    bool valid = b1->bodyType == BodyType::Rigid && b2->bodyType == BodyType::Soft;
    if(!valid)
        return;

    XPBDRigidBody *rb = dynamic_cast<XPBDRigidBody *>(b1);
    XPBDSoftBody *sb = dynamic_cast<XPBDSoftBody *>(b2);
    std::vector<Eigen::Vector3f> &positions = sb->getPositions();

    Geometry *geo = rb->getGeometry();
    if (geo->getType() == eGeometryType::kBox)
    {
        Box *box = dynamic_cast<Box *>(geo);

        Eigen::Vector3f rbx = rb->getPosition();
        Eigen::Quaternionf rbq = rb->getRotation();

        for (int i = 0; i < positions.size(); ++i)
        {
            const Eigen::Vector3f clocal = rbq.inverse() * (positions[i] - rbx);
            Eigen::Vector3f q(0, 0, 0);
            for (unsigned int i = 0; i < 3; ++i)
            {
                q[i] = std::max(-box->dim[i] / 2.0f, std::min(box->dim[i] / 2.0f, clocal[i]));
            }
            if ((clocal - q).norm() < 1e-6f)
            {
                ParticlePos temp;
                temp.id = i;
                temp.pos = clocal;
                particlePos.push_back(temp);
            }
        }
    }

    // TODO
    else if (geo->getType() == eGeometryType::kSphere)
    {
    }
}

XPBDAttatchment::~XPBDAttatchment() {}

void XPBDAttatchment::solveConstraint(float dt)
{
    XPBDSoftBody *sb = dynamic_cast<XPBDSoftBody *>(b2);
    std::vector<Eigen::Vector3f> &positions = sb->getPositions();

    Eigen::Vector3f rbx = b1->getPosition();
    Eigen::Quaternionf rbq = b1->getRotation();

    for (ParticlePos& pp : particlePos)
    {
        const Eigen::Vector3f clocal = rbq.inverse() * (positions[pp.id] - rbx);

        const Eigen::Vector3f dr = pp.pos - clocal;
        float C = dr.norm();

        if(C < 1e-6)
            continue;
        
        float w1 = 1.0f / b1->getMass();
        float w2 = sb->invMass(pp.id);

        float alpha = 0.0f / dt / dt;
        float dlambda = -C / (w1 + w2 + alpha);
        Eigen::Vector3f p = dlambda * dr.normalized();

        b1->setPosition(rbx + p * w1);
        //b2->setPosition(rbx - p * w2);
        positions[pp.id] -= p * w2;
    }
}