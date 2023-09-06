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

    Vector3 com;
    
    float Mass{0.0f};

    // elastic material parameter
    float mu;
    float lambda;

    int VSize{0};
    int FSize{0};

    explicit MeshState(const MeshDataNative udata);
    ~MeshState();
};