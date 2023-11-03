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

    [DllImport(DllName)]
    public static extern unsafe MeshState* InitMeshState(MeshDataNative data);

    [DllImport(DllName)]
    public static extern unsafe TetMeshState* InitTetMeshState(TetMeshDataNative data);

    [DllImport(DllName)]
    public static extern unsafe void DisposeMeshState(MeshState* state);

    [DllImport(DllName)]
    public static extern unsafe void DisposeTetMeshState(TetMeshState* state);

    // Physics.cpp

    [DllImport(DllName, ExactSpelling = true, CharSet = CharSet.Ansi)]
    public static extern unsafe void AddMesh(MeshState* meshState, string path,
        Vector3 pos, Quaternion rot, float mass, float mu, float lambda, int matType);

    [DllImport(DllName)]
    public static extern void AddContact(Vector3 p, Vector3 n, float seperation);


    [DllImport(DllName)]
    public static extern void CreateSoftBody();

    [DllImport(DllName)]
    public static extern void InitSoftBody();

    [DllImport(DllName)]
    public static extern void DeleteSoftBody();

    [DllImport(DllName)]
    public static extern void SimulationUpdate(float dt);

    [DllImport(DllName)]
    public static extern void GetTransform(int index, out Vector3 position, out Quaternion rotation);

    [DllImport(DllName)]
    public static extern void AddTorque(int index, float torque, Vector3 axis);

    [DllImport(DllName)]
    public static extern int AddXPBDRigidBody(Vector3 pos, Quaternion rot, Vector3 inertia, float mass);

    [DllImport(DllName)]
    public static extern int AddXPBDRigidBox(Vector3 pos, Quaternion rot, Vector3 size, float mass);

    [DllImport(DllName)]
    public static extern int AddXPBDRigidCylinder(Vector3 pos, Quaternion rot, float radius, float height, float mass);

    [DllImport(DllName)]
    public static extern unsafe int AddXPBDSoftBody
        (MeshState* meshState, TetMeshState* tetState, Vector3 pos, Quaternion rot, float mass, float mu, float lambda);

    [DllImport(DllName)]
    public static extern void AddPosConstraints(int ID1, int ID2, Vector3 R1, Vector3 R2, float len, float comp);

    [DllImport(DllName)]
    public static extern void AddFixedJoint(int ID1, int ID2);

    [DllImport(DllName)]
    public static extern void AttachRigidSoft(int rId, int sId);

    [DllImport(DllName)]
    public static extern void setBodyMaterial(int ID, float mu, float lambda);

    [DllImport(DllName)]
    public static extern void XPBDSimInit();

    [DllImport(DllName)]
    public static extern void XPBDSimUpdate(float dt, int substeps);

    [DllImport(DllName)]
    public static extern void XPBDSimDelete();

    // IO.cpp
    [DllImport(DllName)]
    public static extern unsafe void ApplyDirtyVis(MeshState* state, MeshDataNative data);

    [DllImport(DllName)]
    public static extern unsafe void ApplyDirtyTet(TetMeshState* state, TetMeshDataNative data);

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
