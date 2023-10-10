#pragma once
#include <vector>
#include <memory>

class XPBDBody;

class XPBDSimulation
{
public:
    XPBDSimulation();
    ~XPBDSimulation();

    int AddBody(XPBDBody* body);
    void SetBodyMaterial(int index, float mu, float lambda);
    void Update(float dt, int substeps);

protected:
    std::vector<std::unique_ptr<XPBDBody>> bodies;
};
