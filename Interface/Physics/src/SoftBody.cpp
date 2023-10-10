#include "SoftBody.h"
#include "Simulation.h"
#include "CollisionSolver.h"

#include <igl/readMESH.h>
#include <igl/AABB.h>
#include <igl/barycentric_coordinates.h>
#include <igl/per_vertex_normals.h>
#include <iostream>
#include <numeric>

#ifndef NODEBUG
using namespace std;
static ofstream logfile;
#endif

SoftBody::SoftBody()
    : simulation(new Simulation())
{
#ifndef NODEBUG
    logfile.open("./log/SoftBody.log");
#endif
}

SoftBody::~SoftBody()
{
    clear();

#ifndef NODEBUG
    logfile << "SoftBody Destruct" << std::endl;
    logfile.flush();
    logfile.close();
#endif
}

void SoftBody::Init()
{
    computeBarycentricCoord();

    generateParticleList();
    generateTetList();
    computeTetVolumes();
    computeOneRingVolumes();
    generateMassMatrix();

    simulation->SetSoftBody(this);
    simulation->Reset();
}

void SoftBody::Update(float dt)
{
    // AddTorque(0, 100.0f, EigenVector3::UnitX());

    simulation->Update(dt);

    UpdateMeshes();

    m_external_force.setZero();
}

void SoftBody::UpdateMeshes()
{
    // current_positions to each tetMesh
    for (int i = 0; i < (int)m_vertices_number; ++i)
    {
        const auto &indexPair = vertexIndexToMeshIndex[i];
        auto &mesh = meshes[indexPair.first]->tetMesh;
        const auto& index = indexPair.second;

        mesh->V.col(index) = m_current_positions.block_vector(i);
    }

    // compute center of mass and move mesh to origin
    computeTranslate();
    

    // compute rotation of each mesh
    computeRotation();

    // tetMesh to visual mesh MeshState
    for(auto& mesh : meshes)
    {
        auto& visV = mesh->state->V;
        auto& tetV = mesh->tetMesh->V;
        //auto& tetT = mesh->tetMesh->T;

        for(int i = 0; i < visV->cols(); ++i)
        {
            const auto& baryCoord = mesh->barycentricCoord[i];
            const auto& closestTet = mesh->closestTet[i];
            EigenVector3 res = EigenVector3::Zero();

            for(int j = 0; j < 4; ++j)
            {
                const EigenVector3 vertex = tetV.col(closestTet[j]);
                res += baryCoord[j] * vertex;
            }
            visV->col(i) = res;
        }
    }

    //UpdateNormals();
}

void SoftBody::Reset()
{
    clear();
    Init();
    // Update();
}

void SoftBody::AddMesh(MeshState *state, const char *path,
                    EigenVector3 pos, Eigen::Quaternionf rot,
                    ScalarType mass, ScalarType mu, ScalarType lambda, int matType)
{
    Eigen::MatrixXf V;
    Eigen::MatrixXi F, T;
    bool success = igl::readMESH(path, V, T, F);
    if (!success)
    {
        fprintf(stderr, "Load file %s error\n", path);
        return;
    }

    // Move tetMesh to origin 
    Eigen::RowVector3f mean = V.colwise().mean();
    V.rowwise() -= mean;

    TetMesh *tm = new TetMesh();
    tm->restV = V.transpose();
    tm->V = tm->restV;
    tm->T = T.transpose();

    Mesh *newMesh = new Mesh(state, tm);
    newMesh->position = pos;
    newMesh->rotation = rot;
    newMesh->Mass = mass;
    newMesh->mu = mu;
    newMesh->lambda = lambda;
    newMesh->materialType = matType;
    meshes.push_back(newMesh);

#ifndef NODEBUG
    logfile << "file: " << path << "\n";
    logfile << "V: " << tm->V.rows() << " " << tm->V.cols() << "\n";
    logfile << tm->V << "\n";
    logfile << "T: " << tm->T.rows() << " " << tm->T.cols() << "\n";
    logfile << tm->T << "\n";

    logfile << "meshstate:\n";
    logfile << "V: " << state->V->rows() << " " << state->V->cols() << "\n";
    logfile << *(state->V) << "\n";

    logfile << "materialType: " << newMesh->materialType << "\n";
    logfile << "mu, lambda: " << newMesh->mu << " " << newMesh->lambda << "\n";
    logfile << "pos: " << newMesh->position.transpose() << "\n";
    logfile << "rot: " << newMesh->rotation << "\n";
    logfile << flush;
#endif
}

void SoftBody::AddContact(EigenVector3 p, EigenVector3 n, ScalarType seperation)
{
    // std::unique_ptr<Contact> contact(new Contact(p, n, seperation, this));
    // if(contact->valid)
    //     m_contacts.push_back(std::move(contact));
}

void SoftBody::AddTorque(int index, float torque, EigenVector3 axis)
{
    Mesh *mesh = meshes[index];
    EigenVector3 com = mesh->position;
    float ang_accel = torque / mesh->Mass;
    for(int i = 0; i < mesh->tetMesh->V.cols(); ++i)
    {
        int globalIndex = meshIndexToVertexIndex[std::make_pair(index, i)];
        EigenVector3 r = current_position(globalIndex) - com;
        r -= r.dot(axis) * axis;

        EigenVector3 force = r.cross(axis) * ang_accel * vertexMass(globalIndex);
        m_external_force.block_vector(globalIndex) += force;
    }
}

void SoftBody::GetMeshTransform(int index, EigenVector3 &pos, Eigen::Quaternionf &rot)
{
    Mesh* mesh = meshes[index];
    pos = mesh->position;
    rot = mesh->rotation;
}

const EigenVector3 SoftBody::current_position(int index) const
{
    EigenVector3 position;
    position[0] = m_current_positions[3 * index + 0];
    position[1] = m_current_positions[3 * index + 1];
    position[2] = m_current_positions[3 * index + 2];

    return position;
}

const EigenVector3 SoftBody::current_velocity(int index) const
{
    EigenVector3 velocity;
    velocity[0] = m_current_velocities[3 * index + 0];
    velocity[1] = m_current_velocities[3 * index + 1];
    velocity[2] = m_current_velocities[3 * index + 2];

    return velocity;
}

const ScalarType SoftBody::vertexMass(int index) const
{
    ScalarType mass = m_mass_matrix_1d.coeff(index, index);
    return mass;
}

const ScalarType SoftBody::vertexMassInv(int index) const
{
    ScalarType mass_inv = m_inv_mass_matrix_1d.coeff(index, index);
    return mass_inv;
}

void SoftBody::clear()
{
    for(auto& mesh : meshes)
        delete mesh;
    meshes.clear();
}

void SoftBody::computeBarycentricCoord()
{
    for (auto &mesh : meshes)
    {
        igl::AABB<Eigen::MatrixXf, 3> tree;
        Eigen::MatrixXf tetV = mesh->tetMesh->V.transpose();
        Eigen::MatrixXi tetT = mesh->tetMesh->T.transpose();
        tree.init(tetV, tetT);

        Eigen::VectorXi I;
        Eigen::VectorXf sqrd;
        Eigen::MatrixXf CP;
        Eigen::MatrixXf Q = mesh->state->V->transpose();

        tree.squared_distance(tetV, tetT, Q, sqrd, I, CP);

        Eigen::MatrixXf A, B, C, D, L;
        A.resize(Q.rows(), 3);
        B.resize(Q.rows(), 3);
        C.resize(Q.rows(), 3);
        D.resize(Q.rows(), 3);

        for (int i = 0; i < Q.rows(); ++i)
        {
            auto &tet = tetT.row(I[i]);
            A.row(i) = tetV.row(tet[0]);
            B.row(i) = tetV.row(tet[1]);
            C.row(i) = tetV.row(tet[2]);
            D.row(i) = tetV.row(tet[3]);
        }

        igl::barycentric_coordinates(Q, A, B, C, D, L);

        mesh->barycentricCoord.clear();
        mesh->barycentricCoord.resize(L.rows());
        for (int i = 0; i < mesh->barycentricCoord.size(); ++i)
        {
            mesh->barycentricCoord[i] = L.row(i).transpose();
        }

        mesh->closestTet.clear();
        mesh->closestTet.resize(L.rows());
        for (int i = 0; i < mesh->closestTet.size(); ++i)
        {
            auto &tet = tetT.row(I[i]);
            mesh->closestTet[i] = tet.transpose();
        }
    }

#ifndef NODEBUG
    for(const auto& mesh : meshes)
    {
        logfile << "mesh->barycentricCoord:\n";
        for (auto &bary : mesh->barycentricCoord)
        {
            logfile << bary.transpose() << "\n";
        }
        logfile << "mesh->closestTet:\n";
        for (auto &tet : mesh->closestTet)
        {
            logfile << tet.transpose() << "\n";
        }
    }
    logfile << flush;
#endif

}

void SoftBody::generateParticleList()
{
    vertexIndexToMeshIndex.clear();
    meshIndexToVertexIndex.clear();

    m_vertices_number = 0;
    for (auto &m : meshes)
    {
        // m->vertexOffset = m_vertices_number;
        m_vertices_number += m->tetMesh->V.cols();
    }
    m_system_dimension = m_vertices_number * 3;

    // Assign initial position, velocity and mass to all the vertices.
    // Assign color to all the vertices.
    m_restpose_positions.resize(m_system_dimension);
    m_current_positions.resize(m_system_dimension);
    m_current_velocities.resize(m_system_dimension);
    m_external_force.resize(m_system_dimension);

    // Assign initial position to all the vertices.

    m_restpose_positions.setZero();
    unsigned int index = 0;
    for (int k = 0; k < meshes.size(); ++k)
    {
        auto &m = meshes[k]->tetMesh;
        auto &translate = meshes[k]->position;
        for (int i = 0; i < m->V.cols(); ++i)
        {
            m_restpose_positions.block_vector(index) = m->V.col(i) + translate;

            std::pair<int, int> mesh_index_pair(k, i);
            vertexIndexToMeshIndex[index] = mesh_index_pair;
            meshIndexToVertexIndex[mesh_index_pair] = index;

            index++;
        }
    }    
    m_current_positions = m_restpose_positions;
    m_previous_positions = m_restpose_positions;

    // Assign initial velocity
    m_current_velocities.setZero();
    m_previous_velocities = m_current_velocities;

    // Set force to zero
    m_external_force.setZero();

#ifndef NODEBUG
    logfile << "vertices num: " << m_vertices_number << "\n";
    logfile << "system dim: " << m_system_dimension << "\n";
    logfile << "vertexIndexToMeshIndex\n";
    for (auto &map : vertexIndexToMeshIndex)
        logfile << map.first << " (" << map.second.first << ", " << map.second.second << ")\n";
    logfile << "meshIndexToVertexIndex\n";
    for (auto &map : meshIndexToVertexIndex)
        logfile << "(" << map.first.first << ", " << map.first.second << ") " << map.second << "\n";
    logfile << "m_restpose_positions: " << m_vertices_number << "\n";
    for (int i = 0; i < m_restpose_positions.size(); i += 3)
    {
        logfile << m_restpose_positions[i + 0] << " ";
        logfile << m_restpose_positions[i + 1] << " ";
        logfile << m_restpose_positions[i + 2] << "\n";
    }
    logfile << "m_current_velocities: " << m_current_velocities.size() / 3 << "\n";
    for (int i = 0; i < m_current_velocities.size(); i += 3)
    {
        logfile << m_current_velocities[i + 0] << " ";
        logfile << m_current_velocities[i + 1] << " ";
        logfile << m_current_velocities[i + 2] << "\n";
    }
    logfile << flush;

    fprintf(stderr, "generateParticleList() done\n");
#endif
}

void SoftBody::generateTetList()
{
    m_tets.clear();

    int tets_number = 0;
    for (int i = 0; i < meshes.size(); ++i)
    {
        tets_number += meshes[i]->tetMesh->T.cols();
    }

    m_tets.reserve(tets_number);
    for (int i = 0; i < meshes.size(); ++i)
    {
        auto &m = meshes[i]->tetMesh;
        // const auto &offset = meshes[i]->vertexOffset;
        for (int j = 0; j < m->T.cols(); ++j)
        {
            const auto tet = m->T.col(j);
            EigenVector4I v;
            for(int k = 0; k < 4; ++k)
            {
                std::pair<int, int> mesh_index_pair(i, tet(k));
                v(k) = meshIndexToVertexIndex[mesh_index_pair];
            }
            m_tets.push_back(v);
        }
    }

    // Set Matrix form tet index
    m_tets_4X.resize(4, m_tets.size());

    for(int i = 0; i < m_tets_4X.cols(); ++i)
    {
        m_tets_4X.col(i) = m_tets[i];
    }
#ifndef NODEBUG
    logfile << "tets: " << m_tets.size() << "\n";
    for (int i = 0; i < m_tets.size(); ++i)
    {
        logfile << m_tets[i].transpose() << "\n";
    }
    logfile << "tets_4X: " << m_tets_4X.cols() << "\n";
    logfile << m_tets_4X << "\n";
    logfile << flush;

    fprintf(stderr, "generateTetList() done\n");
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
        const float mass = meshes[i]->Mass;
        const auto &mesh = meshes[i]->tetMesh;
        const float volume = meshes[i]->volume;
        for (int j = 0; j < mesh->V.cols(); ++j)
        {
            int index = meshIndexToVertexIndex[std::make_pair(i, j)];
            const ScalarType entry = mass * meshes[i]->restOneRingVolumes[j] / volume;

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

#ifndef NODEBUG
    logfile << "mesh mass: " << meshes.size() << "\n";
    for (auto &m : meshes)
        logfile << m->Mass << " ";
    logfile << "\n";
    logfile << "mass matrix size: " << m_mass_matrix.rows() << "\n";
    logfile << "mass matrix 1d size: " << m_mass_matrix_1d.rows() << "\n";
    for (unsigned int i = 0; i != m_mass_matrix_1d.rows(); i++)
        logfile << m_mass_matrix_1d.coeff(i, i) << "\n";
    logfile << "mass matrix sum: " << m_mass_matrix_1d.sum() << "\n";
    logfile << "mass inv matrix size: " << m_inv_mass_matrix.rows() << "\n";
    logfile << "mass inv matrix 1d size: " << m_inv_mass_matrix_1d.rows() << "\n";
    for (unsigned int i = 0; i != m_inv_mass_matrix_1d.rows(); i++)
        logfile << m_inv_mass_matrix_1d.coeff(i, i) << "\n";
    logfile << flush;
    fprintf(stderr, "generateMassMatrix() done\n");
#endif
}

void SoftBody::computeTetVolumes()
{
    for(auto& mesh : meshes)
    {
        auto& tetVolumes = mesh->restTetVolumes;
        tetVolumes.clear();
        tetVolumes.resize(mesh->tetMesh->T.cols());

        for(int i = 0; i < tetVolumes.size(); ++i)
        {
            const auto& tet = mesh->tetMesh->T.col(i);
            EigenVector3 tetVertices[4];
            for (int j = 0; j < 4; ++j)
                tetVertices[j] = mesh->tetMesh->V.col(tet[j]);
            
            const EigenVector3 diff1 = tetVertices[1] - tetVertices[0];
            const EigenVector3 diff2 = tetVertices[2] - tetVertices[0];
            const EigenVector3 diff3 = tetVertices[3] - tetVertices[0];
            tetVolumes[i] = diff3.dot((diff1).cross(diff2)) / 6.0f;
            if(tetVolumes[i] <= 0.0f)
                std::cerr << " ERROR: Bad rest volume found: " << tetVolumes[i] << std::endl;
        }
    }

#ifndef NODEBUG
    for(int i = 0; i < meshes.size(); ++i)
    {
        const auto& mesh = meshes[i];
        logfile << "mesh " << i << " tetVolumes: " << mesh->restTetVolumes.size() << "\n";
        for(const auto& V : mesh->restTetVolumes)
            logfile << V << "\n";
        logfile << "volumes sum: " 
            << std::accumulate(mesh->restTetVolumes.begin(), mesh->restTetVolumes.end(), 0.0f);
    }
    logfile << endl;
    fprintf(stderr, "computeTetVolumes() done\n");
#endif
}

void SoftBody::computeOneRingVolumes()
{
    for(auto& mesh : meshes)
    {
        auto& oneRingVolumes = mesh->restOneRingVolumes;
        oneRingVolumes.clear();
        oneRingVolumes.resize(mesh->tetMesh->V.cols());
        for(int i = 0; i < mesh->restTetVolumes.size(); ++i)
        {
            const ScalarType quarter = 0.25f * mesh->restTetVolumes[i];
            const auto& tet = mesh->tetMesh->T.col(i);
            for (int y = 0; y < 4; ++y)
                oneRingVolumes[tet[y]] += quarter;
        }
        mesh->volume = std::accumulate(mesh->restOneRingVolumes.begin(), mesh->restOneRingVolumes.end(), 0.0f);

        // Compute restpose com
        EigenVector3 vertexSum = EigenVector3::Zero();
        for(int i = 0; i < mesh->restOneRingVolumes.size(); ++i)
        {
            vertexSum += mesh->tetMesh->V.col(i) * mesh->restOneRingVolumes[i];
        }
        mesh->tetMesh->x_cm0 = vertexSum / mesh->volume;
    }

#ifndef NODEBUG
    for(int i = 0; i < meshes.size(); ++i)
    {
        const auto& mesh = meshes[i];
        logfile << "mesh " << i << " restOneRingVolumes: " << mesh->restOneRingVolumes.size() << "\n";
        for(const auto& V : mesh->restOneRingVolumes)
            logfile << V << "\n";
        logfile << "volumes sum: " << mesh->volume << "\n";
        logfile << "x_cm0: " << mesh->tetMesh->x_cm0 << "\n";
    }
    logfile << flush;
    fprintf(stderr, "computeOneRingVolumes() done\n");
#endif
}

void SoftBody::computeTranslate()
{
    for (int i = 0; i < meshes.size(); ++i)
    {
        EigenVector3 com = EigenVector3::Zero();
        const auto& tetV = meshes[i]->tetMesh->V;
        for(int j = 0; j < tetV.cols(); ++j)
        {
            com += meshes[i]->restOneRingVolumes[j] * tetV.col(j);
        }
        com /= meshes[i]->volume;
        meshes[i]->position = com;
        meshes[i]->tetMesh->V.colwise() -= com;
    }
}

void SoftBody::computeRotation()
{
    for(int i = 0; i < meshes.size(); ++i)
    {
        Mesh *mesh = meshes[i];
        const EigenVector3 x_cm0 = mesh->tetMesh->x_cm0;
        const EigenVector3 x_cm = EigenVector3::Zero();

        EigenMatrix3 Apq;
        Apq.setZero();
        for(int x = 0; x < mesh->tetMesh->V.cols(); ++x)
        {
            const EigenVector3 p = mesh->tetMesh->V.col(x) - x_cm;
            const EigenVector3 q = mesh->tetMesh->restV.col(x) - x_cm0;
            Apq += mesh->restOneRingVolumes[x] * (p * q.transpose());
        }

        EigenMatrix3 R,S;
        polarDecomposition(Apq, R, S);
        mesh->rotation = R;

        for(int x = 0; x < mesh->tetMesh->V.cols(); ++x)
        {
            mesh->tetMesh->V.col(x) = mesh->rotation.inverse() * mesh->tetMesh->V.col(x);
        }
    }
    
}

void SoftBody::UpdateNormals()
{
    for (auto &mesh : meshes)
    {
        Eigen::MatrixXf V, N;
        Eigen::MatrixXi F;
        V = mesh->state->V->transpose();
        F = mesh->state->F->transpose();
        igl::per_vertex_normals(V, F, N);

        *(mesh->state->N) = N.transpose();
    }
}

void SoftBody::UpdateAABB()
{
    auto& pos = m_current_positions;

    // V: n*3, T: m*4
    Eigen::MatrixXf V = Eigen::Map<Eigen::MatrixXf>(pos.data(), 3, pos.size() / 3).transpose();
    Eigen::MatrixXi T = m_tets_4X.transpose();

    aabb.init(V,T);
}
