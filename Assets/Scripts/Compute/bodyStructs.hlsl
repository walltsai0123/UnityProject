#ifndef _BODYSTRUCTS_H_
#define _BODYSTRUCTS_H_

#include<./settings.hlsl>
#include<./quaternion.hlsl>

struct SoftBodyParticle
{
    REAL3 pos;
    REAL3 prevPos;
    REAL3 vel;
    REAL invMass;
};

struct RigidBodyData
{
    uint isFixed;
    REAL3 position;
    quaternion rotation;
    REAL invMass;
    REAL3x3 IBodyInv;
};

struct RigidBody
{
    int isFixed;
    REAL3 position;
    REAL3 prevPos;
    quaternion rotation;
    quaternion prevRot;
    REAL3 vel;
    REAL3 omega;
    REAL invMass;
    REAL3x3 IBody;
    REAL3x3 IBodyInv;

};



RWStructuredBuffer<SoftBodyParticle> particles;

#endif