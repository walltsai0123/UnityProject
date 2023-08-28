#include "SoftBody.h"
#include "Debug.h"
#include "Simulation.h"
#include <igl/per_vertex_normals.h>
#include <iostream>

SoftBody::SoftBody()
    : simulation(new Simulation())
{
}

SoftBody::~SoftBody()
{
    clear();
}

void SoftBody::Init()
{
    generateParticleList();
    generateTetList();
    computeTetVolumes();
    computeOneRingVolumes();
    generateMassMatrix();

    simulation->SetSoftBody(this);
    simulation->Reset();
}

void SoftBody::Update()
{
    simulation->Update();
}

void SoftBody::Reset()
{
    clear();
    Init();
    // Update();
}

void SoftBody::AddMesh(MeshState *state)
{
    meshes.push_back(state);
}

void SoftBody::UpdateMeshes()
{
    // current_positions to each Meshstate
    assert(m_vertices_number == m_current_positions.size() / 3);
    for(int i = 0; i < (int)m_vertices_number; ++i)
    {
        const auto& indexPair = vertexIndexToMeshIndex[i];
        auto& mesh = meshes[indexPair.first];
        
        mesh->V->col(indexPair.second) = m_current_positions.block_vector(i);
    }
    
    UpdateNormals();
}

const EigenVector3 SoftBody::current_position(int index) const
{
    assert(index >= 0);
    assert((unsigned)index < m_vertices_number);

    EigenVector3 position;
    position[0] = m_current_positions[3 * index + 0];
    position[1] = m_current_positions[3 * index + 1];
    position[2] = m_current_positions[3 * index + 2];

    return position;
}

const EigenVector3 SoftBody::current_velocity(int index) const
{
    assert(index >= 0);
    assert((unsigned)index < m_vertices_number);

    EigenVector3 velocity;
    velocity[0] = m_current_velocities[3 * index + 0];
    velocity[1] = m_current_velocities[3 * index + 1];
    velocity[2] = m_current_velocities[3 * index + 2];

    return velocity;
}

const ScalarType SoftBody::vertexMass(int index) const
{
    assert(index >= 0);
    assert((unsigned)index < m_vertices_number);

    ScalarType mass = m_mass_matrix_1d.coeff(index, index);
    return mass;
}

const ScalarType SoftBody::vertrxMassInv(int index) const
{
    assert(index >= 0);
    assert((unsigned)index < m_vertices_number);

    ScalarType mass_inv = m_inv_mass_matrix_1d.coeff(index, index);
    return mass_inv;
}

void SoftBody::clear()
{
    meshes.clear();
}
void SoftBody::generateParticleList()
{
    vertexIndexToMeshIndex.clear();
    meshIndexToVertexIndex.clear();

    m_vertices_number = 0;
    for (auto &m : meshes)
    {
        m_vertices_number += m->VSize;
    }
    m_system_dimension = m_vertices_number * 3;
    // ScalarType unit_mass = m_total_mass / m_system_dimension;

    // Assign initial position, velocity and mass to all the vertices.
    // Assign color to all the vertices.
    m_restpose_positions.resize(m_system_dimension);
    m_current_positions.resize(m_system_dimension);
    m_current_velocities.resize(m_system_dimension);

    // Assign initial position to all the vertices.

    m_restpose_positions.setZero();
    unsigned int index = 0;
    for (int k = 0; k < meshes.size(); ++k)
    {
        auto &m = meshes[k];
        for (int i = 0; i < m->VSize; ++i)
        {
            m_restpose_positions.block_vector(index) = m->V->col(i);
            std::pair<int, int> mesh_index_pair(k, i);
            vertexIndexToMeshIndex[index] = mesh_index_pair;
            meshIndexToVertexIndex[mesh_index_pair] = index;
            index++;
        }
    }
    // Assign initial velocity to zero
    m_current_velocities.setZero();
    m_current_positions = m_restpose_positions;
    m_previous_positions = m_restpose_positions;
    m_previous_velocities = m_current_velocities;

#ifndef NDEBUG
    std::cout << "vertexIndexToMeshIndex\n";
    for(auto& map : vertexIndexToMeshIndex)
        std::cout << map.first << " (" << map.second.first << ", " << map.second.second << ")\n";
    std::cout << "meshIndexToVertexIndex\n";
    for(auto& map : meshIndexToVertexIndex)
        std::cout << "(" << map.first.first << ", " << map.first.second << ") " << map.second << "\n";
    std::cout << "m_restpose_positions: " << m_vertices_number << "\n";
    for (int i = 0; i < m_restpose_positions.size(); i += 3)
    {
        std::cout << m_restpose_positions[i + 0] << " ";
        std::cout << m_restpose_positions[i + 1] << " ";
        std::cout << m_restpose_positions[i + 2] << "\n";
    }
    std::cout<< "generateParticleList() done" << std::endl;
#endif
}

void SoftBody::generateTetList()
{
    m_tets.clear();
    tetsOffset.clear();
    tetsOffset.resize(meshes.size());

    int tets_number = 0;
    for (int i = 0; i < meshes.size(); ++i)
    {
        tets_number += meshes[i]->TSize;
    }
    // Set offset
    tetsOffset[0] = 0;
    for (int i = 1; i < meshes.size(); ++i)
    {
        tetsOffset[i] = tetsOffset[i - 1] + meshes[i - 1]->VSize;
    }

    std::cout << "tetsOffset " << tetsOffset.size() << "\n";
    Debug::PrintSTLVectorToLog(tetsOffset);
    std::cout << "tets_number: " << tets_number << "\n";
    m_tets.reserve(tets_number * 4);
    for (int i = 0; i < meshes.size(); ++i)
    {
        auto &m = meshes[i];
        auto &offset = tetsOffset[i];
        for (int j = 0; j < m->TSize; ++j)
        {
            const auto tet = m->T->col(j);
            m_tets.push_back(tet(0) + offset);
            m_tets.push_back(tet(1) + offset);
            m_tets.push_back(tet(2) + offset);
            m_tets.push_back(tet(3) + offset);
        }
    }
#ifndef NDEBUG
    std::cout << "tets: " << m_tets.size() / 4 << "\n";
    for (int i = 0; i < m_tets.size(); i += 4)
    {
        std::cout << m_tets[i + 0] << " ";
        std::cout << m_tets[i + 1] << " ";
        std::cout << m_tets[i + 2] << " ";
        std::cout << m_tets[i + 3] << "\n";
    }
    std::cout<< "generateTetList() done" << std::endl;
#endif
}

void SoftBody::generateMassMatrix()
{
    m_mass_matrix.resize(m_system_dimension, m_system_dimension);
    m_inv_mass_matrix.resize(m_system_dimension, m_system_dimension);
    m_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);
    m_inv_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);
    std::vector<ScalarType> meshVolume;
    meshVolume.resize(meshes.size());

    // Mass Matrix
    std::vector<SparseMatrixTriplet> m_triplets;
    std::vector<SparseMatrixTriplet> m_triplets_1d;
    m_triplets.clear();
    m_triplets_1d.clear();
    for (int i = 0; i < meshes.size(); ++i)
    {
        auto &mesh = meshes[i];
        meshVolume[i] = 0;
        for (int j = tetsOffset[i]; j < mesh->VSize + tetsOffset[i]; ++j)
        {
            meshVolume[i] += m_restOneRingVolumes[j];
        }
        for (int index = tetsOffset[i]; index < mesh->VSize + tetsOffset[i]; ++index)
        {
            const ScalarType entry = mesh->Mass * m_restOneRingVolumes[index] / meshVolume[i];

            m_triplets_1d.push_back(SparseMatrixTriplet(index, index, entry));

            for (int j = 0; j < 3; ++j)
            {
                const IndexType INDEX = 3 * index + j;
                m_triplets.push_back(SparseMatrixTriplet(INDEX, INDEX, entry));
            }
        }
    }
    m_mass_matrix.setFromTriplets(m_triplets.begin(), m_triplets.end());
    m_mass_matrix_1d.setFromTriplets(m_triplets_1d.begin(), m_triplets_1d.end());

    // Mass Inverse Matrix
    std::vector<SparseMatrixTriplet> m_inv_triplets;
    std::vector<SparseMatrixTriplet> m_inv_triplets_1d;
    m_inv_triplets.clear();
    m_inv_triplets_1d.clear();
    for (unsigned int i = 0; i != m_mass_matrix.rows(); i++)
    {
        ScalarType mi = m_mass_matrix.coeff(i, i);
        ScalarType mi_inv;
        if (std::abs(mi) > 1e-12f)
        {
            mi_inv = 1.0f / mi;
        }
        else
        {
            // ugly ugly!
            m_mass_matrix.coeffRef(i, i) = 1e-12f;
            mi_inv = 1e12f;
        }
        m_inv_triplets.push_back(SparseMatrixTriplet(i, i, mi_inv));
    }
    for (unsigned int i = 0; i != m_mass_matrix_1d.rows(); i++)
    {
        ScalarType mi = m_mass_matrix_1d.coeff(i, i);
        ScalarType mi_inv;
        if (std::abs(mi) > 1e-12f)
        {
            mi_inv = 1.0f / mi;
        }
        else
        {
            // ugly ugly!
            m_mass_matrix_1d.coeffRef(i, i) = 1e-12f;
            mi_inv = 1e12f;
        }
        m_inv_triplets_1d.push_back(SparseMatrixTriplet(i, i, mi_inv));
    }
    m_inv_mass_matrix.setFromTriplets(m_inv_triplets.begin(), m_inv_triplets.end());
    m_inv_mass_matrix_1d.setFromTriplets(m_inv_triplets_1d.begin(), m_inv_triplets_1d.end());

#ifndef NDEBUG
    std::cout << "mesh mass: " << meshes.size() << "\n";
    for(auto& m : meshes)
        std::cout << m->Mass << " ";
    std::cout << "\n";
    std::cout << "mesh volume: " << meshVolume.size() << "\n";
    for(auto& V : meshVolume)
        std::cout << V << " ";
    std::cout << "\n";
    std::cout << "mass matrix size: " << m_mass_matrix.rows() << "\n";
    std::cout << "mass matrix 1d size: " << m_mass_matrix_1d.rows() << "\n";
    for (unsigned int i = 0; i != m_mass_matrix_1d.rows(); i++)
        std::cout << m_mass_matrix_1d.coeff(i, i) << "\n";
    std::cout << "mass inv matrix size: " << m_inv_mass_matrix.rows() << "\n";
    std::cout << "mass inv matrix 1d size: " << m_inv_mass_matrix_1d.rows() << "\n";
    for (unsigned int i = 0; i != m_inv_mass_matrix_1d.rows(); i++)
        std::cout << m_inv_mass_matrix_1d.coeff(i, i) << "\n";
    std::cout << "generateMassMatrix() done" << std::endl;;
#endif
}

void SoftBody::computeTetVolumes()
{
    m_restTetVolumes.clear();
    m_restTetVolumes.resize(m_tets.size() / 4);
    for (unsigned int i = 0; i < m_restTetVolumes.size(); ++i)
    {
        int tet[4];
        for (int j = 0; j < 4; ++j)
            tet[j] = m_tets[4 * i + j];

        EigenVector3 tetVertices[4];
        for (int j = 0; j < 4; ++j)
            tetVertices[j] = m_restpose_positions.block_vector(tet[j]);

        const EigenVector3 diff1 = tetVertices[1] - tetVertices[0];
        const EigenVector3 diff2 = tetVertices[2] - tetVertices[0];
        const EigenVector3 diff3 = tetVertices[3] - tetVertices[0];
        m_restTetVolumes[i] = diff3.dot((diff1).cross(diff2)) / 6.0f;

        if (m_restTetVolumes[i] < 0.0f)
        {
            std::cerr << " ERROR: Bad rest volume found: " << m_restTetVolumes[i] << std::endl;
        }
        assert(m_restTetVolumes[i] >= 0.0f);
    }

#ifndef NDEBUG
    std::cout << "m_restTetVolumes: " << m_restTetVolumes.size() << "\n";
    Debug::PrintSTLVectorToLog(m_restTetVolumes);
    std::cout << "computeTetVolumes() done" << std::endl;
#endif
}

void SoftBody::computeOneRingVolumes()
{
    m_restOneRingVolumes.clear();
    m_restOneRingVolumes.resize(m_vertices_number, 0.0f);
    for (unsigned int x = 0; x < m_restTetVolumes.size(); ++x)
    {
        const ScalarType quarter = 0.25f * m_restTetVolumes[x];
        for (int y = 0; y < 4; ++y)
        {
            m_restOneRingVolumes[m_tets[4 * x + y]] += quarter;
        }
    }

#ifndef NDEBUG
    std::cout << "m_restOneRingVolumes: " << m_restOneRingVolumes.size() << "\n";
    Debug::PrintSTLVectorToLog(m_restOneRingVolumes);
    std::cout << "computeOneRingVolumes() done" << std::endl;
#endif
}

void SoftBody::UpdateNormals()
{
    for(auto& mesh : meshes)
    {
        Eigen::MatrixXf V, N;
        Eigen::MatrixXi F;
        V = mesh->V->transpose();
        F = mesh->F->transpose();
        igl::per_vertex_normals(V, F, N);

        *(mesh->N) = N.transpose();
    }
}

