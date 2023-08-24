#include "Physics/SoftBody.h"
#include "Physics/Debug.h"
#include <igl/per_vertex_normals.h>
#include <iostream>

SoftBody::SoftBody()
    : m_mu(5.0f), m_lambda(100.0f)
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
}

void SoftBody::Update()
{
    // assert(_vertices.size() * 3 == m_current_positions.size());
    // for (unsigned i = 0; i < _vertices.size(); ++i)
    // {
    //     _vertices[i] = EigenVector3(m_current_positions[3 * i + 0], m_current_positions[3 * i + 1], m_current_positions[3 * i + 2]);
    // }
}

void SoftBody::Reset()
{
    clear();
    Init();
    Update();
}

void SoftBody::AddMesh(MeshState *state)
{
    meshes.push_back(state);
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

    // m_mass_matrix.resize(m_system_dimension, m_system_dimension);
    // m_inv_mass_matrix.resize(m_system_dimension, m_system_dimension);

    // m_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);
    // m_inv_mass_matrix_1d.resize(m_vertices_number, m_vertices_number);

    // Assign initial position to all the vertices.

    m_restpose_positions.setZero();
    unsigned int index = 0;
    for (auto &m : meshes)
    {
        for (int i = 0; i < m->VSize; ++i)
        {
            m_restpose_positions.block_vector(index) = m->V->col(i);
            index++;
        }
    }
    // Assign initial velocity to zero
    m_current_velocities.setZero();
    m_current_positions = m_restpose_positions;
    m_previous_positions = m_restpose_positions;
    m_previous_velocities = m_current_velocities;

#ifndef NDEBUG
    std::cout << "m_restpose_positions: " << m_vertices_number << "\n";
    for (int i = 0; i < m_restpose_positions.size(); i += 3)
    {
        std::cout << m_restpose_positions[i + 0] << " ";
        std::cout << m_restpose_positions[i + 1] << " ";
        std::cout << m_restpose_positions[i + 2] << "\n";
    }
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
    std::cout << std::endl;
#endif
}

void SoftBody::UpdateNormals()
{
}
