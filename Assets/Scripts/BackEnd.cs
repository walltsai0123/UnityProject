using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

public class BackEnd
{
    private const string DllName = "__cpp-interface";

    public static readonly VertexAttributeDescriptor[] VertexBufferLayout = new[]
    {
        new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, 0),
        new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3, 1),
        new VertexAttributeDescriptor(VertexAttribute.Color, VertexAttributeFormat.Float32, 3, 2),
        new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2, 3), // UV
        new VertexAttributeDescriptor(VertexAttribute.TexCoord1, VertexAttributeFormat.UInt32, 1, 2) // Selection coupled with Color
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

    [DllImport(DllName)]
    public static extern unsafe MeshState* InitMeshState(MeshDataNative data);

    [DllImport(DllName)]
    public static extern unsafe void AddMesh(MeshState* meshState);

    [DllImport(DllName)]
    public static extern unsafe void DisposeMeshState(MeshState* state);

    [DllImport(DllName)]
    public static extern void CreateSoftBody();

    [DllImport(DllName)]
    public static extern void InitSoftBody();

    [DllImport(DllName)]
    public static extern void DeleteSoftBody();

    [DllImport(DllName)]
    public static extern void SimulationUpdate();

    [DllImport(DllName)]
    public static extern void MeshesUpdate();

    // IO.cpp
    [DllImport(DllName)]
    public static extern unsafe void ApplyDirty(MeshState* state, MeshDataNative data);

    [DllImport(DllName, ExactSpelling = true, CharSet = CharSet.Ansi)]
    public static extern unsafe bool ReadMESH(string path,
        out float* VPtr, out int VSize,
        out float* NPtr, out int NSize,
        out uint* FPtr, out int FSize,
        out uint* TPtr, out int TSize);
}
