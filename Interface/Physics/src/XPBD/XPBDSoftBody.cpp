#include "XPBD/XPBDSoftBody.h"

#include <igl/AABB.h>
#include <igl/barycentric_coordinates.h>
#include <igl/per_vertex_normals.h>
#include <igl/readMESH.h>

int XPBDSoftBody::counter = 0;

namespace
{
    void svd_rv(const Eigen::Matrix3f &F, Eigen::Matrix3f &U, Eigen::Vector3f Sigma, Eigen::Matrix3f &V)
    {
        const Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        V = svd.matrixV();
        Sigma = svd.singularValues();

        Eigen::Matrix3f L = Eigen::Matrix3f::Identity();
        L(2, 2) = (U * V.transpose()).determinant();

        const float detU = U.determinant();
        const float detV = V.determinant();

        if (detU < 0.0 && detV > 0)
            U = U * L;
        if (detU > 0.0 && detV < 0.0)
            V = V * L;

        Sigma[2] = Sigma[2] * L(2, 2);
    }
    void polarDecomposition(const Eigen::Matrix3f &A, Eigen::Matrix3f &R, Eigen::Matrix3f &S)
    {
        Eigen::Matrix3f U, V;
        Eigen::Vector3f Sigma;
        svd_rv(A, U, Sigma, V);

        R = U * V.transpose();
        S = V * Sigma.asDiagonal() * V.transpose();
    }
}

XPBDSoftBody::XPBDSoftBody(Eigen::Vector3f pos, Eigen::Quaternionf rot, MeshState *state, TetMeshState *tetState, float Mass, float mu, float lambda)
    : XPBDBody(BodyType::Soft),
      m_state(state), m_tetState(tetState),
      restX(pos),
      m_mu(mu),
      m_lambda(lambda)
{
    // Set logfile
    std::string logfileName = "log/XPBDSoftBody_" + std::to_string(counter) + ".log";
    logfile.open(logfileName);

    x = pos;
    q = rot;
    mass = Mass;

    Eigen::MatrixXf tetV = tetState->V->transpose();
    Eigen::MatrixXi tetT = tetState->T->transpose();

    computeSkinningInfo(tetV, tetT);
    initPhysics(tetV, tetT);

    counter++;
}

XPBDSoftBody::~XPBDSoftBody()
{
    counter--;
    logfile.flush();
    logfile.close();
}

void XPBDSoftBody::setMaterial(float mu, float lambda)
{
    m_mu = mu;
    m_lambda = lambda;
}

void XPBDSoftBody::collectCollsion(float dt)
{
    m_collisions.clear();
    Eigen::Vector3f surfaceN(0.0f, 1.0f, 0.0f);
    for(int i = 0; i < m_vertices_num; ++i)
    {
        Eigen::Vector3f predPos = m_positions[i] + m_velocities[i] * dt;
        if(predPos.y() <= 0.0f)
        {
            Collision newCollision;
            newCollision.index = i;
            newCollision.surfaceN = surfaceN;
            newCollision.vn_ = surfaceN.dot(m_velocities[i]);
            if(m_positions[i].y() > 0.0f)
            {
                Eigen::Vector3f ray = predPos - m_positions[i];
                float coef = -m_positions[i].y() / ray.y();
                Eigen::Vector3f q = m_positions[i] + coef * ray;
                newCollision.q = q;
            }
            else if(m_positions[i].y() <= 0.0f)
            {
                Eigen::Vector3f q = predPos;
                q.y() = 0.0f;
                newCollision.q = q;
            }
            m_collisions.push_back(newCollision);
        }
    }

    // logfile << "\nCollision\n";
    // for(int i = 0; i < m_collisions.size(); ++i)
    // {
    //     const Collision &c = m_collisions[i];
    //     logfile << "index: " << c.index;
    //     logfile << " q: " << c.q.transpose();
    //     logfile << " surfaceN: " << c.surfaceN.transpose();
    //     logfile << std::endl;
    // }
}

void XPBDSoftBody::preSolve(float dt)
{
    const Eigen::Vector3f gravity(0.0f, -9.81f, 0.0f);
    for (int i = 0; i < m_vertices_num; ++i)
    {
        // Ignore zero mass
        if (m_invMass[i] == 0)
            continue;

        const Eigen::Vector3f accel = gravity;

        m_velocities[i] += accel * dt;
        m_prevPositions[i] = m_positions[i];
        m_positions[i] += m_velocities[i] * dt;
    }

}

void XPBDSoftBody::solve(float dt)
{
    solveElements(dt);
    solveCollision(dt);

    // Easy collision
    // for (int i = 0; i < m_vertices_num; ++i)
    // {
    //     float planeY = 0.0f;
    //     if (m_positions[i].y() < planeY)
    //     {
    //         m_positions[i].y() = planeY;

    //         // simple friction
    //         float fricton = 1000.0f;
    //         Eigen::Vector3f F = m_prevPositions[i] - m_positions[i];
    //         m_positions[i].x() += F(0) * std::min(1.0f, dt * fricton);
    //         m_positions[i].z() += F(2) * std::min(1.0f, dt * fricton);
    //     }
    // }
}

void XPBDSoftBody::postSolve(float dt)
{
    for (int i = 0; i < m_vertices_num; ++i)
    {
        // Ignore zero mass
        if (m_invMass[i] == 0)
            continue;

        m_velocities[i] = (m_positions[i] - m_prevPositions[i]) / dt;
    }
}

void XPBDSoftBody::velocitySolve(float dt)
{
    float frictionCoef = 0.4f;
    float restitutionCoef;
    for(Collision &c : m_collisions)
    {
        Eigen::Vector3f v = m_velocities[c.index];
        float vn = c.surfaceN.dot(v);
        Eigen::Vector3f vt = v - vn * c.surfaceN;

        // tangent
        Eigen::Vector3f dvt = -vt.normalized() * std::min(dt * frictionCoef * c.fn.norm(), vt.norm());
        m_velocities[c.index] += dvt;

        // normal
        restitutionCoef = (std::abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
        Eigen::Vector3f dvn = c.surfaceN * (-vn + std::min(0.0f, -restitutionCoef * c.vn_));
        m_velocities[c.index] += dvn;
    }
}

void XPBDSoftBody::endFrame()
{
    updateTetMesh();
    updateVisMesh();
}


// private function
void XPBDSoftBody::computeSkinningInfo(const Eigen::MatrixXf &tetV, const Eigen::MatrixXi &tetT)
{
    int VSize = m_state->VSize;
    m_skinningInfo.resize(VSize);

    igl::AABB<Eigen::MatrixXf, 3> tree;
    tree.init(tetV, tetT);

    Eigen::VectorXi I;
    Eigen::VectorXf sqrd;
    Eigen::MatrixXf CP;
    Eigen::MatrixXf Q = m_state->V->transpose();
    tree.squared_distance(tetV, tetT, Q, sqrd, I, CP);

    Eigen::MatrixXf A, B, C, D, L;
    A.resize(VSize, 3);
    B.resize(VSize, 3);
    C.resize(VSize, 3);
    D.resize(VSize, 3);

    for (int i = 0; i < Q.rows(); ++i)
    {
        auto &tet = tetT.row(I[i]);
        A.row(i) = tetV.row(tet[0]);
        B.row(i) = tetV.row(tet[1]);
        C.row(i) = tetV.row(tet[2]);
        D.row(i) = tetV.row(tet[3]);
    }
    igl::barycentric_coordinates(Q, A, B, C, D, L);

    for (int i = 0; i < VSize; ++i)
    {
        m_skinningInfo[i](0) = I[i];
        m_skinningInfo[i](1) = L.row(i)(1);
        m_skinningInfo[i](2) = L.row(i)(2);
        m_skinningInfo[i](3) = L.row(i)(3);
    }

#ifndef NODEBUG
    logfile << "Skinning Info\n";
    for (int i = 0; i < VSize; ++i)
    {
        const Eigen::Vector4f &skInfo = m_skinningInfo[i];
        logfile << i << " vertex pos: " << Q.row(i) << "\n";
        logfile << "(";
        logfile << skInfo(0) << ", ";
        logfile << skInfo(1) << ", ";
        logfile << skInfo(2) << ", ";
        logfile << skInfo(3) << ")\n";

        const Eigen::RowVector4i tet = tetT.row(skInfo(0));
        logfile << "tet vertex pos:" << tet << "\n";
        logfile << tetV.row(tet(0)) << "\n";
        logfile << tetV.row(tet(1)) << "\n";
        logfile << tetV.row(tet(2)) << "\n";
        logfile << tetV.row(tet(3)) << "\n";
    }
    logfile.flush();
#endif
}

void XPBDSoftBody::initPhysics(const Eigen::MatrixXf &tetV, const Eigen::MatrixXi &tetT)
{
    m_vertices_num = tetV.rows();
    m_tets_num = tetT.rows();

    // Set positions and velocities
    m_positions.resize(m_vertices_num);
    m_localPositions.resize(m_vertices_num);
    m_prevPositions.resize(m_vertices_num);
    m_restPositions.resize(m_vertices_num);
    m_velocities.resize(m_vertices_num, Eigen::Vector3f::Zero());
    m_cacheVelocities.resize(m_vertices_num);
    for (int i = 0; i < m_vertices_num; ++i)
    {
        const Eigen::Vector3f pos = tetV.row(i).transpose() + x;
        m_restPositions[i] = pos;
        m_positions[i] = pos;
        m_localPositions[i] = pos - x;
    }

    // Set tets
    m_tets.resize(m_tets_num);
    for (int i = 0; i < m_tets_num; ++i)
    {
        m_tets[i] = tetT.row(i).transpose();
    }

    // Set inverse mass
    m_invMass.resize(m_vertices_num, 0.0f);

    // Compute rest pose , mass , rest volume
    float totalVolume = 0.0f;
    m_invDm.resize(m_tets_num);
    m_tetVolumes.resize(m_tets_num);
    for (int i = 0; i < m_tets_num; ++i)
    {
        int id0 = m_tets[i](0);
        int id1 = m_tets[i](1);
        int id2 = m_tets[i](2);
        int id3 = m_tets[i](3);

        Eigen::Matrix3f RestPose;
        RestPose.col(0) = m_positions[id1] - m_positions[id0];
        RestPose.col(1) = m_positions[id2] - m_positions[id0];
        RestPose.col(2) = m_positions[id3] - m_positions[id0];
        m_invDm[i] = RestPose.inverse();

        float V = RestPose.determinant() / 6.0f;
        float partialV = V / 4.0f;
        m_invMass[id0] += partialV;
        m_invMass[id1] += partialV;
        m_invMass[id2] += partialV;
        m_invMass[id3] += partialV;

        m_tetVolumes[i] = V;
        totalVolume += V;
    }

    // scale mass with density and inverse it
    float density = mass / totalVolume;
    for (int i = 0; i < m_vertices_num; ++i)
    {
        m_invMass[i] *= density;
        if (m_invMass[i] != 0.0f)
            m_invMass[i] = 1.0f / m_invMass[i];
    }

    // move COM to x
    Eigen::Vector3f COM = Eigen::Vector3f::Zero();
    for (int i = 0; i < m_vertices_num; ++i)
    {
        COM += m_positions[i] / m_invMass[i];
    }
    COM /= mass;
    Eigen::Vector3f delta = x - COM;
    if (delta.norm() >= 1e-6)
    {
        for (int i = 0; i < m_vertices_num; ++i)
        {
            m_positions[i] += delta;
            m_localPositions[i] += delta;
            m_restPositions[i] += delta;
        }
    }

#ifndef NODEBUG
    logfile << "Init Physics\n";
    logfile << "vertices num: " << m_vertices_num << "\n";
    logfile << "tets num: " << m_tets_num << "\n";
    logfile << "x: " << x.transpose() << "\n";
    logfile << "COM: " << COM.transpose() << "\n";
    logfile << "Positions: " << m_positions.size() << "\n";
    for (const auto &pos : m_positions)
        logfile << pos.transpose() << "\n";
    logfile << "Tets: " << m_tets.size() << "\n";
    for (int i = 0; i < m_tets.size(); ++i)
        logfile << m_tets[i].transpose() << "\n";
    logfile << "Inverse mass: " << m_invMass.size() << "\n";
    for (const auto &invM : m_invMass)
        logfile << invM << "\n";
    logfile << "Tet Volumes: " << m_tetVolumes.size() << "\n";
    for (const auto &V : m_tetVolumes)
        logfile << V << "\n";
    logfile << "Density: " << density << "\n";
    logfile << "totalVolume: " << totalVolume << "\n";
    logfile << "Total_mass: " << mass << "\n";
    logfile << "mu lambda: " << m_mu << " " << m_lambda << "\n";
    logfile << "1/mu 1/lambda: " << 1.0f/m_mu << " " << 1.0f/m_lambda << "\n";
    logfile.flush();
#endif
}

void XPBDSoftBody::solveElements(float dt)
{
    for (int i = 0; i < m_tets_num; ++i)
    {
        float devCompliance = 1.0f / m_mu / m_tetVolumes[i];
        solveDeviatoric(i, devCompliance, dt);

        float volCompliance = 1.0f / m_lambda / m_tetVolumes[i];
        solveVolumetric(i, volCompliance, dt);
    }
}

Eigen::Matrix3f XPBDSoftBody::getDeformationGradient(int index)
{
    int i = index;
    int id0 = m_tets[i](0);
    int id1 = m_tets[i](1);
    int id2 = m_tets[i](2);
    int id3 = m_tets[i](3);

    Eigen::Matrix3f Ds;
    Ds.setZero();
    Ds.col(0) = m_positions[id1] - m_positions[id0];
    Ds.col(1) = m_positions[id2] - m_positions[id0];
    Ds.col(2) = m_positions[id3] - m_positions[id0];

    return Ds * m_invDm[i];
}

void XPBDSoftBody::solveDeviatoric(int index, float compliance, float dt)
{
    // C = trace(FTF) - 3
    const Eigen::Matrix3f F = getDeformationGradient(index);
    const float trFTF = F.col(0).squaredNorm() + F.col(1).squaredNorm() + F.col(2).squaredNorm();
    float C = sqrtf(trFTF);

    if (C == 0.0f)
        return;

    Eigen::MatrixXf gradient;
    gradient.setZero(3, 4);

    gradient.col(1) += 2 * F.col(0) * m_invDm[index](0, 0);
    gradient.col(1) += 2 * F.col(1) * m_invDm[index](0, 1);
    gradient.col(1) += 2 * F.col(2) * m_invDm[index](0, 2);

    gradient.col(2) += 2 * F.col(0) * m_invDm[index](1, 0);
    gradient.col(2) += 2 * F.col(1) * m_invDm[index](1, 1);
    gradient.col(2) += 2 * F.col(2) * m_invDm[index](1, 2);

    gradient.col(3) += 2 * F.col(0) * m_invDm[index](2, 0);
    gradient.col(3) += 2 * F.col(1) * m_invDm[index](2, 1);
    gradient.col(3) += 2 * F.col(2) * m_invDm[index](2, 2);

    gradient.col(0) = -gradient.col(1) - gradient.col(2) - gradient.col(3);

    gradient /= 2 * sqrtf(trFTF);

    float weight = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        int id = m_tets[index](i);
        weight += gradient.col(i).squaredNorm() * m_invMass[id];
    }

    if (weight == 0.0f)
        return;

    float h2 = dt * dt;
    float alpha = compliance / h2;
    float dlambda = -C / (weight + alpha);

    for (int i = 0; i < 4; ++i)
    {
        int id = m_tets[index](i);
        m_positions[id] += m_invMass[id] * gradient.col(i) * dlambda;
    }
}

void XPBDSoftBody::solveVolumetric(int index, float compliance, float dt)
{
    // C = det(F) - 1
    const Eigen::Matrix3f F = getDeformationGradient(index);
    const float gamma = 1.0f + m_mu / m_lambda;
    float C = F.determinant() - gamma;
    if (C == 0.0f)
        return;

    // F = [ f1 | f2 | f3 ], dF = [ f2xf3 | f3xf1 | f1xf2 ]
    Eigen::Matrix3f dF;
    dF.col(0) = F.col(1).cross(F.col(2));
    dF.col(1) = F.col(2).cross(F.col(0));
    dF.col(2) = F.col(0).cross(F.col(1));

    Eigen::MatrixXf gradient;
    gradient.setZero(3, 4);

    gradient.col(1) += dF.col(0) * m_invDm[index](0, 0);
    gradient.col(1) += dF.col(1) * m_invDm[index](0, 1);
    gradient.col(1) += dF.col(2) * m_invDm[index](0, 2);

    gradient.col(2) += dF.col(0) * m_invDm[index](1, 0);
    gradient.col(2) += dF.col(1) * m_invDm[index](1, 1);
    gradient.col(2) += dF.col(2) * m_invDm[index](1, 2);

    gradient.col(3) += dF.col(0) * m_invDm[index](2, 0);
    gradient.col(3) += dF.col(1) * m_invDm[index](2, 1);
    gradient.col(3) += dF.col(2) * m_invDm[index](2, 2);

    gradient.col(0) = -gradient.col(1) - gradient.col(2) - gradient.col(3);

    float weight = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        int id = m_tets[index](i);
        weight += gradient.col(i).squaredNorm() * m_invMass[id];
    }

    if (weight == 0.0f)
        return;

    float alpha = compliance / dt / dt;
    float dlambda = -C / (weight + alpha);

    for (int i = 0; i < 4; ++i)
    {
        int id = m_tets[index](i);
        m_positions[id] += m_invMass[id] * gradient.col(i) * dlambda;
    }
}

void XPBDSoftBody::generateCollision()
{
    m_collisions.clear();
    Eigen::Vector3f surfaceN(0.0f, 1.0f, 0.0f);
    for(int i = 0; i < m_vertices_num; ++i)
    {
        if(m_positions[i].y() <= 0.0f)
        {
            Collision newCollision;
            newCollision.index = i;
            newCollision.surfaceN = surfaceN;
            newCollision.vn_ = surfaceN.dot(m_velocities[i]);
            if(m_prevPositions[i].y() > 0.0f)
            {
                Eigen::Vector3f ray = m_positions[i] - m_prevPositions[i];
                float coef = -m_prevPositions[i].y() / ray.y();
                Eigen::Vector3f q = m_prevPositions[i] + coef * ray;
                newCollision.q = q;
            }
            else if(m_prevPositions[i].y() <= 0.0f)
            {
                Eigen::Vector3f q = m_positions[i];
                q.y() = 0.0f;
                newCollision.q = q;
            }
            m_collisions.push_back(newCollision);
        }
    }
}

void XPBDSoftBody::solveCollision(float dt)
{
    for(Collision &c : m_collisions)
    {
        float C = (m_positions[c.index] - c.q).dot(c.surfaceN);
        if(C >= 0.0f)
            continue;

        float alpha = 0.0f;
        float w1 = m_invMass[c.index];
        float w2 = 0.0f;
        float dlambda = -C / (w1 + w2 + alpha);
        Eigen::Vector3f p = dlambda * c.surfaceN;
        c.fn = p / dt / dt;
        m_positions[c.index] += p * w1;

        Eigen::Vector3f dp = m_positions[c.index] - m_prevPositions[c.index];
        Eigen::Vector3f dp_t = dp - dp.dot(c.surfaceN) * c.surfaceN;

        float C2 = dp_t.norm();
        if(C2 <= 1e-6f)
            continue;
        
        float dlambda_t = -C2 / (w1 + w2 + alpha);
        dlambda_t = std::min(dlambda_t, dlambda * c.frictionCoef);
        Eigen::Vector3f p2 = dlambda_t * dp_t.normalized();
        m_positions[c.index] += p2 * w1;
    }
}

void XPBDSoftBody::updatePos()
{
    // Calculate center of mass
    Eigen::Vector3f newPos = Eigen::Vector3f::Zero();
    for (int i = 0; i < m_vertices_num; ++i)
    {
        newPos += m_positions[i] / m_invMass[i];
    }
    newPos /= mass;
    x = newPos;
}

void XPBDSoftBody::updateRotation()
{
    const Eigen::Vector3f x_cm0 = restX;
    const Eigen::Vector3f x_cm = x;

    Eigen::Matrix3f Apq;
    Apq.setZero();
    for (int i = 0; i < m_vertices_num; ++i)
    {
        const Eigen::Vector3f p = m_positions[i] - x_cm;
        const Eigen::Vector3f q = m_restPositions[i] - x_cm0;
        Apq += (p * q.transpose()) / m_invMass[i];
    }

    Eigen::Matrix3f R, S;
    polarDecomposition(Apq, R, S);
    q = R;
}

void XPBDSoftBody::updateInertia()
{
    Ibody.setZero();

    for (int i = 0; i < m_vertices_num; ++i)
    {
        Eigen::Vector3f ri = m_localPositions[i];
        Eigen::Matrix3f ri_tilde;
        ri_tilde << 0.0f, -ri.z(), ri.y(),
            ri.z(), 0.0f, -ri.x(),
            -ri.y(), ri.x(), 0.0f;
        Ibody += ri_tilde * ri_tilde.transpose() / m_invMass[i];
    }

    I = q * Ibody * q.inverse();
}

void XPBDSoftBody::updateLocalPos()
{
    for (int i = 0; i < m_vertices_num; ++i)
    {
        m_localPositions[i] = q.inverse() * (m_positions[i] - x);
    }
}

void XPBDSoftBody::updateGlobalPos()
{
    for (int i = 0; i < m_vertices_num; ++i)
    {
        m_positions[i] = q * m_localPositions[i] + x;
    }
}

void XPBDSoftBody::updateTetMesh()
{
    for (int i = 0; i < m_vertices_num; ++i)
    {
        m_tetState->V->col(i) = m_positions[i] - x;
    }
}

void XPBDSoftBody::updateVisMesh()
{
    int numVisVerts = m_state->VSize;

    for (int i = 0; i < numVisVerts; ++i)
    {
        int tetId = m_skinningInfo[i](0);

        float b3 = m_skinningInfo[i](3);
        float b2 = m_skinningInfo[i](2);
        float b1 = m_skinningInfo[i](1);
        float b0 = 1.0f - b1 - b2 - b3;

        int id0 = m_tets[tetId](0);
        int id1 = m_tets[tetId](1);
        int id2 = m_tets[tetId](2);
        int id3 = m_tets[tetId](3);

        Eigen::Vector3f newPos = Eigen::Vector3f::Zero();
        newPos += m_tetState->V->col(id0) * b0;
        newPos += m_tetState->V->col(id1) * b1;
        newPos += m_tetState->V->col(id2) * b2;
        newPos += m_tetState->V->col(id3) * b3;

        m_state->V->col(i) = newPos;
    }
}
