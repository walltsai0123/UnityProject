#pragma once

#include "math_headers.h"

#include <fstream>

class SoftBody;

class Contact
{
public:
    Contact(const EigenVector3 &_p, const EigenVector3 &_n, float _phi, SoftBody *sb, int vIndex);
    ~Contact();

    void PrintInfo(std::ofstream& out);

    EigenVector3 p;
    // Contact normal
    EigenVector3 n;
    // Contact tangent 1 & 2
    EigenVector3 t1, t2;

    // Friction coefficient
    float mu;
    // Contact penetration
    float pene;

    // Contact Jacobian Matrix
    // J0: static object (remains zero)
    // J1[4]: tet vertices
    EigenMatrixXX J0;
    EigenMatrixXX J1;
    EigenMatrixXX J0Minv;
    EigenMatrixXX J1Minv;

    SoftBody *softbody;
    // index of vertices
    int vertexIndex;
    
    // Contact gap function
    VectorX phi;
    // Constraint impulse
    VectorX lambda;

    float k;                    // Contact stiffness (Baumgarte stabilization)
    float b;                    // Contact damping (Baumgarte stabilization)
protected:
    explicit Contact();

private:
    void computeContactFrame();
    void computeJacobian();
};

