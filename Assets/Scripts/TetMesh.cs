using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public unsafe class TetMesh : MonoBehaviour
{
    private MeshFilter meshFilter;
    public Mesh mesh { get; private set; }
    public MeshRenderer meshRenderer { get; private set; }
    public MeshData DataRowMajor { get; private set; }
    public MeshState* state;

    public string tetFileName;
    public float mass, mu, lambda;
    public bool meshDirty;


    private void Awake()
    {
        Initialize();
        MeshManager.get.AddTetMesh(this);

        Debug.Log("TetMesh Awake");
    }
    public void Initialize()
    {
        mass = 1.0f;
        mu = 5.0f;
        lambda = 100.0f;

        meshDirty = false;

        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        meshRenderer = GetComponent<MeshRenderer>();

        DataRowMajor = new MeshData(this);

        state = BackEnd.InitMeshState(DataRowMajor.GetNative());
    }
    void Start()
    {
        Debug.Log("TetMesh Start");
    }
    private void Update()
    {
        DataRowMajor.ApplyDirty(state);
        DataRowMajor.ApplyDirtyToMesh(mesh);
        transform.position = DataRowMajor.com;
    }
    private void OnDestroy()
    {
        Dispose();
        Debug.Log("TetMesh Destroy");
    }
    private void Dispose()
    {
        if(DataRowMajor != null)
        {
            DataRowMajor.Dispose();
        }
        BackEnd.DisposeMeshState(state);
    }
}
