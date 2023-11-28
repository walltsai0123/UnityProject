#pragma once

#include <fstream>
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
    
    void AddFixedJoint(int id1, int id2);
    void AttachRigidSoft(int rId, int sId);
    void SetBodyMaterial(int index, float mu, float lambda);
    void Update(float dt, int substeps);

protected:
    std::vector<std::unique_ptr<XPBDBody>> bodies;
    std::vector<std::unique_ptr<XPBDConstraint>> constraints;

private:
    std::ofstream logfile;
};
