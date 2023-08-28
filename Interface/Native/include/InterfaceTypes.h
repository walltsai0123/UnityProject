#pragma once

struct MeshDataNative
{
    float* VPtr;
    float* NPtr;
    float* CPtr;
    float* UVPtr;
    int* FPtr;
    int* TPtr;

    float Mass;

    float mu;
    float lambda;
    
    int VSize;
    int FSize;
    int TSize;
};