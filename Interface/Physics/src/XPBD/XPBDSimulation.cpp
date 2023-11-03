#include "XPBD/XPBDSimulation.h"

#include "XPBD/XPBDAttatchment.h"
#include "XPBD/XPBDBody.h"
#include "XPBD/XPBDPosConstraint.h"
#include "XPBD/XPBDFixedJoint.h"
#include "XPBD/XPBDSimulation.h"


XPBDSimulation::XPBDSimulation()
{
    logfile.open("./log/XPBDSimulation.log");
}

XPBDSimulation::~XPBDSimulation()
{
    bodies.clear();
    constraints.clear();

    logfile.flush();
    logfile.close();
}

int XPBDSimulation::AddBody(XPBDBody *body)
{
    std::unique_ptr<XPBDBody> ptr(body);
    bodies.push_back(std::move(ptr));

    logfile << "Add Body " << bodies.size() - 1 << std::endl;
    return bodies.size() - 1;
}

void XPBDSimulation::GetBodyTransform(int index, Eigen::Vector3f &pos, Eigen::Quaternionf& rot)
{
    pos = bodies[index]->getPosition();
    rot = bodies[index]->getRotation();
}
void XPBDSimulation::AddPosConstraint(int id1, int id2, Eigen::Vector3f r1, Eigen::Vector3f r2, float Length, float comp)
{
    XPBDBody *b1 = bodies[id1].get();
    XPBDBody *b2 = bodies[id2].get();
    constraints.push_back(std::make_unique<XPBDPosConstraint>(b1, b2, r1, r2, Length, comp));
}

void XPBDSimulation::AddFixedJoint(int id1, int id2)
{
    XPBDBody *b1 = bodies[id1].get();
    XPBDBody *b2 = bodies[id2].get();
    constraints.push_back(std::make_unique<XPBDFixedJoint>(b1, b2));
}

void XPBDSimulation::AttachRigidSoft(int rId, int sId)
{
    XPBDBody *b1 = bodies[rId].get();
    XPBDBody *b2 = bodies[sId].get();
    constraints.push_back(std::make_unique<XPBDAttatchment>(b1, b2));
}

void XPBDSimulation::SetBodyMaterial(int index, float mu, float lambda)
{
    bodies[index]->setMaterial(mu, lambda);
}

void XPBDSimulation::Update(float dt, int substeps)
{
    logfile << "Update Start: " << dt << " " << substeps << std::endl;
    float sdt = dt / substeps;
    for(int step = 0; step < substeps; ++step)
    {
        for(auto& body : bodies)
            body->preSolve(sdt);
        //logfile << step << " Pre solve " << std::flush;

        for(auto& body : bodies)
            body->solve(sdt);
        //logfile << "Solve " << std::flush;

        for(auto& C : constraints)
            C->solveConstraint(sdt);
        //logfile << "Constraint solve " << std::flush;

        for(auto& body : bodies)
            body->postSolve(sdt);
        //logfile << "Post solve\n" << std::flush;
    }

    for(auto& body : bodies)
        body->endFrame();

    logfile << "Update End" << std::endl;
}
