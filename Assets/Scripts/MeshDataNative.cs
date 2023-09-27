using System.Runtime.InteropServices;
using UnityEngine;

[StructLayout(LayoutKind.Sequential)]
public readonly unsafe struct MeshDataNative
{
    public readonly float* VPtr;
    public readonly float* NPtr;
    public readonly int* FPtr;

    public readonly Vector3 com;

    public readonly float Mass;

    public readonly int materialType;
    public readonly float Mu;
    public readonly float Lambda;

    public readonly int VSize;
    public readonly int FSize;

    public MeshDataNative(float* vPtr, float* nPtr, int* fPtr, Vector3 COM,
        float mass, int mType, float mu, float lambda, int vSize, int fSize)
    {
        VPtr = vPtr;
        NPtr = nPtr;
        FPtr = fPtr;

        com = COM;

        materialType = mType;
        Mass = mass;
        Mu = mu;
        Lambda = lambda;

        VSize = vSize;
        FSize = fSize;
    }
}
