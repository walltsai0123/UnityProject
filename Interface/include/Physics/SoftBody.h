#pragma once

#include "math_headers.h"
#include "MeshState.h"
#include <string>
#include <memory>
#include <map>

// forward declaration
class Simulation;

class SoftBody
{
    friend class Simulation;

public:
    struct Tet
    {
        unsigned int id1, id2, id3, id4;
        Tet() {}
        Tet(int a, int b, int c, int d) : id1(a), id2(b), id3(c), id4(d) {}
    };

    SoftBody();
    ~SoftBody();
    void Init();
    void Update();
    void Reset();

    void AddMesh(MeshState *state);

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
    std::vector<MeshState*> meshes;
    std::vector<int> tetsOffset;

    unsigned int m_vertices_number;              // m
    unsigned int m_system_dimension;             // 3m
    unsigned int m_expanded_system_dimension;    // 6s
    unsigned int m_expanded_system_dimension_1d; // 2s

    // vertices positions/previous positions/mass
    EigenVector3 m_default_position;
    VectorX m_restpose_positions;   // 1x3m
    VectorX m_current_positions;    // 1x3m
    VectorX m_current_velocities;   // 1x3m
    VectorX m_previous_positions;   // 1x3m
    VectorX m_previous_velocities;  // 1x3m
    SparseMatrix m_mass_matrix;     // 3mx3m
    SparseMatrix m_inv_mass_matrix; // 3mx3m

    SparseMatrix m_mass_matrix_1d;
    SparseMatrix m_inv_mass_matrix_1d;

    ScalarType m_total_mass;

    ScalarType m_mu;
    ScalarType m_lambda;
    ScalarType _laplacian_coeff;

    std::vector<int> m_tets;
private:
    void clear();

    void generateParticleList();
    void generateTetList();

    void UpdateNormals();

};