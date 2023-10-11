#pragma once

#include "XPBDBody.h"
#include "MeshState.h"

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>

class XPBDSoftBody : public XPBDBody
{
public:
    XPBDSoftBody(MeshState *state, const std::string tetMeshFile, Eigen::Vector3f pos, Eigen::Quaternionf rot, float mass = 1.0f, float mu = 100.0f, float lambda = 100.0f);
    ~XPBDSoftBody();

    virtual void setMaterial(float mu, float lambda);

    virtual void preSolve(float dt) override;
    virtual void solve(float dt) override;
    virtual void postSolve(float dt) override; 
    virtual void endFrame() override;

    virtual void translatePos(Eigen::Vector3f delta) override;

protected:
    int m_vertices_num;
    int m_tets_num;

    // Visual Mesh State
    MeshState *m_state;

    // Physics mesh data
    std::vector<Eigen::Vector3f> m_positions;
    std::vector<Eigen::Vector3f> m_prevPositions;
    std::vector<Eigen::Vector4i> m_tets;

    // velocities
    std::vector<Eigen::Vector3f> m_velocities;

    // Skinning info mapping visual mesh to tetMesh
    // Data: (tetid, b1, b2, b3); b0 = 1-b1-b2-b3
    std::vector<Eigen::Vector4f> m_skinningInfo;

    // Rest Pose matrix inverse
    std::vector<Eigen::Matrix3f> m_invDm;

    // mass inverse
    std::vector<float> m_invMass;

    // tet volumes
    std::vector<float> m_tetVolumes;

    float m_mu, m_lambda;

private:
    static int counter;
    std::ofstream logfile;

    void computeSkinningInfo(const Eigen::MatrixXf& tetV, const Eigen::MatrixXi& tetT);
    void initPhysics(const Eigen::MatrixXf& tetV, const Eigen::MatrixXi& tetT);
    void solveElements(float dt);
    Eigen::Matrix3f getDeformationGradient(int index);
    void solveDeviatoric(int index, float compliance, float dt);
    void solveVolumetric(int index, float compliance, float dt);
    void updatePos();
    void updateTetMesh();
    void updateVisMesh();

};

