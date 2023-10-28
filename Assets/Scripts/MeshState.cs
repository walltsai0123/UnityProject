using System.Runtime.InteropServices;
using UnityEngine;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct MeshState
{
    public readonly void* VPtr;
    public readonly void* NPtr;
    public readonly void* FPtr;

    public readonly int VSize;
    public readonly int FSize;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct TetMeshState
{
    public readonly void* VPtr;
    public readonly void* TPtr;

    public readonly int VSize;
    public readonly int TSize;
}