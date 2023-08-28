using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct MeshState
{
    public readonly void* VPtr;
    public readonly void* NPtr;
    public readonly void* CPtr;
    public readonly void* UVPtr;
    public readonly void* FPtr;
    public readonly void* TPtr;

    public readonly float Mass;
    public readonly float mu;
    public readonly float lambda;
    public readonly int VSize;
    public readonly int FSize;
    public readonly int TSize;
}
