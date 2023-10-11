#pragma once
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

class XPBDBody;
class XPBDConstraint;

class XPBDSimulation
{
public:
    XPBDSimulation();
    ~XPBDSimulation();

    int AddBody(XPBDBody* body);
    void GetBodyTransform(int index, Eigen::Vector3f& pos, Eigen::Quaternionf& rot);
    void AddPosConstraint(int id1, int id2, Eigen::Vector3f r1, Eigen::Vector3f r2, float Length, float comp);
    void SetBodyMaterial(int index, float mu, float lambda);
    void Update(float dt, int substeps);

protected:
    std::vector<std::unique_ptr<XPBDBody>> bodies;
    std::vector<std::unique_ptr<XPBDConstraint>> constraints;
};
