#pragma once
#include "InterfaceTypes.h"
#include <Eigen/Core>

struct MeshState
{
    // Matrices are column-major
    // dimensions 3 * VSize
    Eigen::MatrixXf* V;

    // dimensions 3 * VSize
    Eigen::MatrixXf* N;

    // dimensions 4 * VSize
    Eigen::MatrixXf* C;

    // dimensions 2 * VSize
    Eigen::MatrixXf* UV;

    // dimensions 3 * FSize
    Eigen::MatrixXi* F;

    // dimensions 4 * TSize
    Eigen::MatrixXi* T;

    float Mass{0.0f};

    // elastic material parameter
    float mu;
    float lambda;

    int VSize{0};
    int FSize{0};
    int TSize{0};

    explicit MeshState(const MeshDataNative udata);
    ~MeshState();
};