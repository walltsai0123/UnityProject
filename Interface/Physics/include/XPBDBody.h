#pragma once

class XPBDBody
{
public:
    XPBDBody(){}
    virtual ~XPBDBody(){}

    virtual void setMaterial(float mu, float lambda) = 0;

    virtual void preSolve(float dt) = 0;
    virtual void solve(float dt) = 0;
    virtual void postSolve(float dt) = 0;
    virtual void endFrame() = 0;
};
