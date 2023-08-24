#pragma once

struct MeshDataNative
{
    float* VPtr;
    float* NPtr;
    float* CPtr;
    float* UVPtr;
    int* FPtr;
    int* TPtr;

    int VSize;
    int FSize;
    int TSize;
};