#include "XPBDSimulation.h"

#include "XPBDBody.h"

XPBDSimulation::XPBDSimulation()
{
}

XPBDSimulation::~XPBDSimulation()
{
    bodies.clear();
}

int XPBDSimulation::AddBody(XPBDBody *body)
{
    std::unique_ptr<XPBDBody> ptr(body);
    bodies.push_back(std::move(ptr));

    return bodies.size() - 1;
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
        for(auto& body : bodies)
            body->postSolve(sdt);
    }

    for(auto& body : bodies)
        body->endFrame();
}
