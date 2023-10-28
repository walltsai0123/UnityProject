using System.Runtime.InteropServices;
using UnityEngine;

[StructLayout(LayoutKind.Sequential)]
public readonly unsafe struct MeshDataNative
{
    public readonly float* VPtr;
    public readonly float* NPtr;
    public readonly int* FPtr;

    public readonly int VSize;
    public readonly int FSize;

    public MeshDataNative(float* vPtr, float* nPtr, int* fPtr, int vSize, int fSize)
    {
        VPtr = vPtr;
        NPtr = nPtr;
        FPtr = fPtr;

        VSize = vSize;
        FSize = fSize;
    }
}

[StructLayout(LayoutKind.Sequential)]
public readonly unsafe struct TetMeshDataNative
{
    public readonly float* VPtr;
    public readonly int* TPtr;

    public readonly int VSize;
    public readonly int TSize;

    public TetMeshDataNative(float* vPtr, int* tPtr, int vSize, int tSize)
    {
        VPtr = vPtr;
        TPtr = tPtr;

        VSize = vSize;
        TSize = tSize;
    }
}
