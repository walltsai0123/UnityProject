#include "XPBD/XPBDSimulation.h"

#include "XPBD/XPBDBody.h"
#include "XPBD/XPBDPosConstraint.h"
#include "XPBD/XPBDSimulation.h"

XPBDSimulation::XPBDSimulation()
{
}

XPBDSimulation::~XPBDSimulation()
{
    bodies.clear();
    constraints.clear();
}

int XPBDSimulation::AddBody(XPBDBody *body)
{
    std::unique_ptr<XPBDBody> ptr(body);
    bodies.push_back(std::move(ptr));

    return bodies.size() - 1;
}

void XPBDSimulation::GetBodyTransform(int index, Eigen::Vector3f &pos, Eigen::Quaternionf& rot)
{
    pos = bodies[index]->x;
    rot = bodies[index]->q;
}
void XPBDSimulation::AddPosConstraint(int id1, int id2, Eigen::Vector3f r1, Eigen::Vector3f r2, float Length, float comp)
{
    XPBDBody *b1 = bodies[id1].get();
    XPBDBody *b2 = bodies[id2].get();
    constraints.push_back(std::make_unique<XPBDPosConstraint>(b1, b2, r1, r2, Length, comp));
}

void XPBDSimulation::SetBodyMaterial(int index, float mu, float lambda)
{
    bodies[index]->setMaterial(mu, lambda);
}

void XPBDSimulation::Update(float dt, int substeps)
{
    float sdt = dt / substeps;
    for(int step = 0; step < substeps; ++step)
    {
        for(auto& body : bodies)
            body->preSolve(sdt);
        for(auto& body : bodies)
            body->solve(sdt);
        for(auto& C : constraints)
            C->solveConstraint(sdt);
        for(auto& body : bodies)
            body->postSolve(sdt);
    }

    for(auto& body : bodies)
        body->endFrame();
}
