#pragma once

#include "XPBDBody.h"
#include "MeshState.h"

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>


class XPBDSoftBody : public XPBDBody
{
    struct Collision{
        int index;
        float frictionCoef = 0.4f;
        Eigen::Vector3f q;
        Eigen::Vector3f surfaceN;
        Eigen::Vector3f fn;
        float vn_;
    };
public:
    XPBDSoftBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, MeshState *state, TetMeshState *tetState,  float mass = 1.0f, float mu = 100.0f, float lambda = 100.0f);
    ~XPBDSoftBody();

    inline std::vector<Eigen::Vector3f>& getPositions() { return m_positions; }
    inline float invMass(int index) { return m_invMass[index]; }

    virtual void setMaterial(float mu, float lambda);

    virtual void collectCollsion(float dt) override;
    virtual void preSolve(float dt) override;
    virtual void solve(float dt) override;
    virtual void postSolve(float dt) override;
    virtual void velocitySolve(float dt) override;
    virtual void endFrame() override;


protected:
    int m_vertices_num;
    int m_tets_num;
    
    Eigen::Vector3f restX;

    // Visual Mesh State
    MeshState *m_state;
    // Physic Mesh State;
    TetMeshState *m_tetState;

    // Physics mesh data
    std::vector<Eigen::Vector3f> m_positions;
    std::vector<Eigen::Vector3f> m_localPositions;
    std::vector<Eigen::Vector3f> m_prevPositions;
    std::vector<Eigen::Vector3f> m_restPositions;
    std::vector<Eigen::Vector4i> m_tets;

    // velocities
    std::vector<Eigen::Vector3f> m_cacheVelocities;
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

    // collision
    std::vector<Collision> m_collisions;

    float m_mu, m_lambda;

private:
    static int counter;

    void computeSkinningInfo(const Eigen::MatrixXf& tetV, const Eigen::MatrixXi& tetT);
    void initPhysics(const Eigen::MatrixXf& tetV, const Eigen::MatrixXi& tetT);
    void solveElements(float dt);
    Eigen::Matrix3f getDeformationGradient(int index);

    // Inner constraint solve
    void solveDeviatoric(int index, float compliance, float dt);
    void solveVolumetric(int index, float compliance, float dt);
    void generateCollision();
    void solveCollision(float dt);

    // update the body position and rotation
    void updatePos();
    void updateRotation();
    void updateInertia();

    // update particle positions (global to local)/(local to global)
    void updateLocalPos();
    void updateGlobalPos();

    // Endframe mesh update
    void updateTetMesh();
    void updateVisMesh();
};

