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
    public int[] tets;


    private void Awake()
    {
        Initialize();
    }
    public void Initialize()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        meshRenderer = GetComponent<MeshRenderer>();

        DataRowMajor = new MeshData(this);
        state = BackEnd.InitMeshState(DataRowMajor.GetNative());
    }
    void Start()
    {
        MeshManager.get.AddTetMesh(this);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void OnDestroy()
    {
        Dispose(); 
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
