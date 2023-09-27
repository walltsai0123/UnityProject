using System.Runtime.InteropServices;
using UnityEngine;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct MeshState
{
    public readonly void* VPtr;
    public readonly void* NPtr;
    public readonly void* FPtr;

    public readonly Vector3 com;

    public readonly float Mass;

    public readonly int materialType;
    public readonly float mu;
    public readonly float lambda;

    public readonly int VSize;
    public readonly int FSize;
}
