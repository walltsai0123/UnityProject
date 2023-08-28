#pragma once

#include "math_headers.h"
#include "MeshState.h"
#include <string>
#include <memory>
#include <map>
#include <utility>

// forward declaration
class Simulation;

class SoftBody
{
    friend class Simulation;

public:
    SoftBody();
    ~SoftBody();

    void Init();
    void Update();
    void Reset();

    void AddMesh(MeshState *state);
    
    void UpdateMeshes();

    // accessors
    const int DOFs() const { return m_system_dimension; };
    const VectorX &current_positions() const { return m_current_positions; };
    VectorX &current_positions()             { return m_current_positions; };
    const VectorX &current_velocities() const { return m_current_velocities; };
    VectorX &current_velocities()             { return m_current_velocities; };
    const SparseMatrix &mass_matrix() const { return m_mass_matrix; };
    const SparseMatrix &inv_mass_matrix() const { return m_inv_mass_matrix; };
    const SparseMatrix &mass_matrix_1d() const { return m_mass_matrix_1d; };

    const EigenVector3 current_position(int index) const;
    const EigenVector3 current_velocity(int index) const;
    const ScalarType vertexMass(int index) const;
    const ScalarType vertrxMassInv(int index) const;


protected:
    std::unique_ptr<Simulation> simulation;

    std::vector<MeshState*> meshes;
    std::vector<int> tetsOffset;

    // Global vertex index map to local meshes index: globalVertexIndex->(meshIndex, vertexIndex)
    std::map<int, std::pair<int, int>> vertexIndexToMeshIndex;
    // Local meshes index map to global vertex index: (meshIndex, vertexIndex)->globalVertexIndex
    std::map<std::pair<int, int>, int> meshIndexToVertexIndex;

    unsigned int m_vertices_number;              // m
    unsigned int m_system_dimension;             // 3m
    //unsigned int m_expanded_system_dimension;    // 6s
    //unsigned int m_expanded_system_dimension_1d; // 2s

    // vertices positions/previous positions/mass
    std::vector<int> m_tets;
    VectorX m_restpose_positions;   // 1x3m
    VectorX m_current_positions;    // 1x3m
    VectorX m_current_velocities;   // 1x3m
    VectorX m_previous_positions;   // 1x3m
    VectorX m_previous_velocities;  // 1x3m
    SparseMatrix m_mass_matrix;     // 3mx3m
    SparseMatrix m_inv_mass_matrix; // 3mx3m
    SparseMatrix m_mass_matrix_1d; // mxm
    SparseMatrix m_inv_mass_matrix_1d; // mxm

    // Volume
    std::vector<ScalarType> m_restTetVolumes;
    std::vector<ScalarType> m_restOneRingVolumes;

    
private:
    void clear();

    void generateParticleList();
    void generateTetList();
    void generateMassMatrix();

    void computeTetVolumes();
    void computeOneRingVolumes();

    void UpdateNormals();

};