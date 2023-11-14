#include "XPBD/XPBDAttachment.h"

#include "XPBD/XPBDRigidBody.h"
#include "XPBD/XPBDSoftBody.h"

XPBDAttachment::XPBDAttachment(XPBDBody *B1, XPBDBody *B2)
    : XPBDConstraint(B1, B2, 0.0f)
{
    valid = body1->bodyType == BodyType::Rigid && body2->bodyType == BodyType::Soft;

    logfile << "XPBDAttatchment\n";
    logfile << "valid: " << valid << std::endl;
    if(!valid)
        return;

    XPBDRigidBody *rb = dynamic_cast<XPBDRigidBody *>(body1);
    XPBDSoftBody *sb = dynamic_cast<XPBDSoftBody *>(body2);
    std::vector<Eigen::Vector3f> &positions = sb->getPositions();

    Geometry *geo = rb->getGeometry();
    Eigen::Vector3f rbx = rb->getPosition();
    Eigen::Quaternionf rbq = rb->getRotation();
    if (geo->getType() == eGeometryType::kBox)
    {
        Box *box = dynamic_cast<Box *>(geo);

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

    else if (geo->getType() == eGeometryType::kCylinder)
    {
        Cylinder *cylinder = dynamic_cast<Cylinder *>(geo);

        for (int i = 0; i < positions.size(); ++i)
        {
            const Eigen::Vector3f clocal = rbq.inverse() * (positions[i] - rbx);
            float h = std::abs(clocal.y());
            float r = sqrtf(clocal.x() * clocal.x() + clocal.z() * clocal.z());
            if (h < cylinder->height && r < cylinder->radius)
            {
                ParticlePos temp;
                temp.id = i;
                temp.pos = clocal;
                particlePos.push_back(temp);
            }
        }
    }

    logfile << particlePos.size() << std::endl;
}

XPBDAttachment::~XPBDAttachment() {}

void XPBDAttachment::solveConstraint(float dt)
{
    if(!valid)
        return;
    
    XPBDSoftBody *sb = dynamic_cast<XPBDSoftBody *>(body2);
    std::vector<Eigen::Vector3f> &positions = sb->getPositions();

    Eigen::Vector3f rbx = body1->getPosition();
    Eigen::Quaternionf rbq = body1->getRotation();

    for (ParticlePos& pp : particlePos)
    {
        const Eigen::Vector3f clocal = rbq.inverse() * (positions[pp.id] - rbx);

        const Eigen::Vector3f dr = pp.pos - clocal;
        float C = dr.norm();

        if(C < 1e-6)
            continue;
        
        float w1 = 1.0f / body1->getMass();
        float w2 = sb->invMass(pp.id);

        float alpha = 0.0f / dt / dt;
        float dlambda = -C / (w1 + w2 + alpha);
        Eigen::Vector3f p = dlambda * dr.normalized();
        p = rbq * p;

        body1->setPosition(rbx + p * w1);
        positions[pp.id] -= p * w2;
    }
}