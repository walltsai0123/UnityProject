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

    // dimensions 3 * FSize
    Eigen::MatrixXi* F;

    int VSize{0};
    int FSize{0};

    explicit MeshState(const MeshDataNative udata);
    ~MeshState();
};

struct TetMeshState
{
    // Matrices are column-major
    // dimensions 3 * VSize
    Eigen::MatrixXf* V;

    // dimensions 3 * FTSize
    Eigen::MatrixXi* T;

    int VSize{0};
    int TSize{0};

    explicit TetMeshState(const TetMeshDataNative udata);
    ~TetMeshState();
};