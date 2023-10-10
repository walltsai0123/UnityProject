#include "math_headers.h"

void Vector3mx1ToMatrixmx3(const VectorX &x, EigenMatrixx3 &m)
{
    for (unsigned int i = 0; i < m.rows(); i++)
    {
        m.block<1, 3>(i, 0) = x.block_vector(i).transpose();
    }
}

void Matrixmx3ToVector3mx1(const EigenMatrixx3 &m, VectorX &x)
{
    for (unsigned int i = 0; i < m.rows(); i++)
    {
        x.block_vector(i) = m.block<1, 3>(i, 0).transpose();
    }
}

EigenVector9 flatten(const EigenMatrix3 &A)
{
    EigenVector9 column;

    unsigned int index = 0;
    for (unsigned int j = 0; j < A.cols(); j++)
        for (unsigned int i = 0; i < A.rows(); i++, index++)
            column[index] = A(i, j);

    return column;
}

void EigenMakeSparseIdentityMatrix(unsigned int rows, unsigned int cols, SparseMatrix &I)
{
    assert(rows == cols);
    std::vector<SparseMatrixTriplet> triplets;
    for (unsigned int i = 0; i != rows; i++)
    {
        triplets.push_back(SparseMatrixTriplet(i, i, 1));
    }
    I.resize(rows, cols);
    I.setFromTriplets(triplets.begin(), triplets.end());
}

void polarDecomposition(const EigenMatrix3 &A, EigenMatrix3 &R, EigenMatrix3 &S)
{
    EigenMatrix3 U, V;
    EigenVector3 Sigma;
    svd_rv(A, U, Sigma, V);

    R = U * V.transpose();
    S = V * Sigma.asDiagonal() * V.transpose();
}

void svd_rv(const EigenMatrix3 &F, EigenMatrix3 &U, EigenVector3 Sigma, EigenMatrix3 &V)
{
    const Eigen::JacobiSVD<EigenMatrix3, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    Sigma = svd.singularValues();

    EigenMatrix3 L = EigenMatrix3::Identity();
    L(2, 2) = (U * V.transpose()).determinant();

    const ScalarType detU = U.determinant();
    const ScalarType detV = V.determinant();

    if (detU < 0.0 && detV > 0)
        U = U * L;
    if (detU > 0.0 && detV < 0.0)
        V = V * L;

    Sigma[2] = Sigma[2] * L(2, 2);
}

// // LBFGS
// QueueLBFGS::QueueLBFGS(unsigned int vector_size, unsigned int queue_size)
// {
//     m_data_s = new ScalarType[vector_size * queue_size];
//     m_data_y = new ScalarType[vector_size * queue_size];

//     m_is_empty = true;
//     m_is_full = false;

//     m_vector_size = vector_size;
//     m_capacity = queue_size;
//     m_head_pointer = 0;
//     m_tail_pointer = 0;
// }

// QueueLBFGS::~QueueLBFGS()
// {
//     delete[] m_data_s;
//     delete[] m_data_y;
// }

// int QueueLBFGS::size()
// {
//     if (m_is_empty)
//     {
//         return 0;
//     }
//     else if (m_is_full)
//     {
//         return m_capacity;
//     }
//     else if (m_tail_pointer >= m_head_pointer)
//     {
//         return m_tail_pointer - m_head_pointer;
//     }
//     else
//     {
//         return m_tail_pointer + m_capacity - m_head_pointer;
//     }
//     return 0;
// }

// void QueueLBFGS::enqueue(const VectorX &sk, const VectorX &yk)
// {
//     if (!m_is_full && m_capacity != 0)
//     {
//         ScalarType *p_s_start = m_data_s + m_tail_pointer * m_vector_size;
//         ScalarType *p_y_start = m_data_y + m_tail_pointer * m_vector_size;

//         memcpy(p_s_start, sk.data(), sk.size() * sizeof(ScalarType));
//         memcpy(p_y_start, yk.data(), yk.size() * sizeof(ScalarType));

//         m_tail_pointer = (m_tail_pointer + 1) % m_capacity;
//         m_is_empty = false;
//         checkFull();
//     }
// }

// void QueueLBFGS::dequeue()
// {
//     if (!m_is_empty && m_capacity != 0)
//     {
//         m_head_pointer = (m_head_pointer + 1) % m_capacity;
//         m_is_full = false;
//         checkEmpty();
//     }
// }

// void QueueLBFGS::visitSandY(ScalarType **s, ScalarType **y, int i)
// {
//     int visit_pointer = (i + m_head_pointer) % m_capacity;

//     (*s) = m_data_s + visit_pointer * m_vector_size;
//     (*y) = m_data_y + visit_pointer * m_vector_size;
// }
