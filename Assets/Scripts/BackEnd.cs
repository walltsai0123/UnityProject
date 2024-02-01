using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

public class BackEnd
{
    private const string DllName = "__cpp-interface";

    public static readonly VertexAttributeDescriptor[] VertexBufferLayout = new[]
    {
        new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3),
        new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3),
        new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2), // UV
    };

    static BackEnd()
    {
        Initialize();
        Debug.Log("BackEnd Init");
    }

    // BackEnd.h
    public static void Initialize()
    {
        Initialize(BackEndCallbacks.DebugLog, BackEndCallbacks.DebugLogWarning, BackEndCallbacks.DebugLogError);
    }

    [DllImport(DllName, ExactSpelling = true, CharSet = CharSet.Ansi)]
    private static extern void Initialize(
        BackEndCallbacks.StringCallback debugCallback,
        BackEndCallbacks.StringCallback debugWarningCallback,
        BackEndCallbacks.StringCallback debugErrorCallback);

    [DllImport(DllName, ExactSpelling = true, CharSet = CharSet.Ansi)]
    public static extern unsafe bool ReadMESH(string path,
        out float* VPtr, out int VSize,
        out float* NPtr, out int NSize,
        out uint* FPtr, out int FSize,
        out uint* TPtr, out int TSize);

    [DllImport(DllName, ExactSpelling = true, CharSet = CharSet.Ansi)]
    public static extern unsafe bool ReadPLY(string path,
        out float* VPtr, out int VSize,
        out float* NPtr, out int NSize,
        out uint* FPtr, out int FSize,
        out uint* UVPtr, out int UVSize);
}
