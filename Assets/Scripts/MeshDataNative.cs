
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public readonly unsafe struct MeshDataNative
{
    public readonly float* VPtr;
    public readonly float* NPtr;
    public readonly float* CPtr;
    public readonly float* UVPtr;
    public readonly int* FPtr;
    public readonly int* TPtr;

    public readonly int VSize;
    public readonly int FSize;
    public readonly int TSize;

    public MeshDataNative(float* vPtr, float* nPtr, float* cPtr, float* uvPtr, int* fPtr, int* tPtr,
        int vSize, int fSize, int tSize)
    {
        VPtr = vPtr;
        NPtr = nPtr;
        CPtr = cPtr;
        UVPtr = uvPtr;
        FPtr = fPtr;
        TPtr = tPtr;
        VSize = vSize;
        FSize = fSize;
        TSize = tSize;
    }
}
