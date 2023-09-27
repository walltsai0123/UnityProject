#pragma once

#include "math_headers.h"
#include "MeshState.h"
#include "Contact.h"
#include <string>
#include <memory>
#include <map>
#include <utility>
#include <igl/AABB.h>

// forward declaration
class Simulation;
class CollisionSolver;

class SoftBody
{
    friend class Contact;
    friend class Simulation;
    struct TetMesh
    {
        Eigen::MatrixXf V;
        Eigen::MatrixXi T;
    };
    class Mesh
    {
    public:
        Mesh(MeshState *s, TetMesh *tm) : state(s), tetMesh(tm) {};
        ~Mesh()
        {
            delete tetMesh;
        }
        MeshState *state;
        TetMesh *tetMesh;

        // int vertexOffset;
        std::vector<EigenVector4> barycentricCoord;
        std::vector<EigenVector4I> closestTet;
        std::vector<ScalarType> restTetVolumes;
        std::vector<ScalarType> restOneRingVolumes;
        ScalarType volume;
    };

public:
    SoftBody();
    ~SoftBody();

    void Init();
    void Update(float dt);
    void UpdateMeshes();
    void Reset();

    void AddMesh(MeshState *state, const char *path);
    void AddContact(EigenVector3 p, EigenVector3 n, ScalarType seperation);

    // accessors
    const int DOFs() const { return m_system_dimension; };
    const VectorX &current_positions() const { return m_current_positions; };
    VectorX &current_positions() { return m_current_positions; };
    const VectorX &current_velocities() const { return m_current_velocities; };
    VectorX &current_velocities() { return m_current_velocities; };
    const SparseMatrix &mass_matrix() const { return m_mass_matrix; };
    const SparseMatrix &inv_mass_matrix() const { return m_inv_mass_matrix; };
    const SparseMatrix &mass_matrix_1d() const { return m_mass_matrix_1d; };

    const EigenVector3 current_position(int index) const;
    const EigenVector3 current_velocity(int index) const;
    const ScalarType vertexMass(int index) const;
    const ScalarType vertexMassInv(int index) const;

    // const std::vector<Contact*> getContacts() const;
    // const std::vector<Contact*> getVertexContacts(int index) const;
protected:
    std::unique_ptr<Simulation> simulation;

    std::vector<Mesh*> meshes;

    // Global vertex index map to local meshes index: globalVertexIndex->(meshIndex, vertexIndex)
    std::map<int, std::pair<int, int>> vertexIndexToMeshIndex;
    // Local meshes index map to global vertex index: (meshIndex, vertexIndex)->globalVertexIndex
    std::map<std::pair<int, int>, int> meshIndexToVertexIndex;
    
    unsigned int m_vertices_number;  // m
    unsigned int m_system_dimension; // 3m
    // unsigned int m_expanded_system_dimension;    // 6s
    // unsigned int m_expanded_system_dimension_1d; // 2s

    // vertices positions/previous positions/mass
    std::vector<EigenVector4I> m_tets;
    Eigen::Matrix4Xi m_tets_4X;
    VectorX m_restpose_positions;      // 1x3m
    VectorX m_current_positions;       // 1x3m
    VectorX m_current_velocities;      // 1x3m
    VectorX m_previous_positions;      // 1x3m
    VectorX m_previous_velocities;     // 1x3m
    SparseMatrix m_mass_matrix;        // 3mx3m
    SparseMatrix m_inv_mass_matrix;    // 3mx3m
    SparseMatrix m_mass_matrix_1d;     // mxm
    SparseMatrix m_inv_mass_matrix_1d; // mxm

    igl::AABB<Eigen::MatrixXf, 3> aabb;

private:
    void clear();

    void computeBarycentricCoord();
    void generateParticleList();
    void generateTetList();
    void generateMassMatrix();

    void computeTetVolumes();
    void computeOneRingVolumes();

    void UpdateNormals();
    void UpdateAABB();

    void collisionDetect();
};