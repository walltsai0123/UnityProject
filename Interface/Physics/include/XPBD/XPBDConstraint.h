#pragma once

#include "XPBDBody.h"

#include <fstream>

class XPBDConstraint
{
public:
    XPBDConstraint(XPBDBody* B1, XPBDBody* B2, float comp);
    virtual ~XPBDConstraint();

    virtual void preSolve(){
        lambda = 0.0f;
    }
    virtual void solveConstraint(float dt) = 0;

protected:
    XPBDBody* body1;
    XPBDBody* body2;
    
    bool valid;
    float lambda;
    float compliance;
    
    std::ofstream logfile;
    static int counter;

    void solvePosConstraint(float dt, const Eigen::Vector3f r1, const Eigen::Vector3f r2, float dmax, float comp);
    void solveAngConstraint(float dt, const Eigen::Vector3f dq, float angle, float comp);
};
