using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public unsafe class TetMesh : MonoBehaviour
{
    public enum MaterialType
    {
        MATERIAL_TYPE_COROT,
        MATERIAL_TYPE_StVK,
        MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG,
        MATERIAL_TYPE_TOTAL_NUM
    }
    private MeshFilter meshFilter;
    public Mesh mesh { get; private set; }
    public MeshRenderer meshRenderer { get; private set; }
    public MeshData DataRowMajor { get; private set; }
    public MeshState* state;

    public string tetFileName;
    public MaterialType materialType = MaterialType.MATERIAL_TYPE_NEOHOOKEAN_EXTEND_LOG;
    public float mass = 1f, mu = 5f, lambda = 100f;
    public bool meshDirty;

    private int ID;


    private void Awake()
    {
        Initialize();
        // ID = MeshManager.get.AddTetMesh(this);

        Debug.Log("TetMesh Awake");
    }
    public void Initialize()
    {
        meshDirty = false;

        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        meshRenderer = GetComponent<MeshRenderer>();

        DataRowMajor = new MeshData(this);

        state = BackEnd.InitMeshState(DataRowMajor.GetNative());

        ID = BackEnd.AddXPBDSoftBody(state, tetFileName, mass, mu, lambda);
        Debug.Log(ID);
    }
    void Start()
    {
        Debug.Log("TetMesh Start");
    }
    private void FixedUpdate()
    {
        BackEnd.setBodyMaterial(ID, mu, lambda);

        DataRowMajor.ApplyDirty(state);
        DataRowMajor.ApplyDirtyToMesh(mesh);
        // this.GetComponent<MeshCollider>().sharedMesh = mesh;
        //BackEnd.GetTransform(ID, out Vector3 pos, out Quaternion rot);

        //transform.SetPositionAndRotation(pos, rot);
    }
    private void Update()
    {
        //float dir = Input.GetAxis("Vertical");
        //float torque = 100f;

        //BackEnd.AddTorque(ID, dir * torque, Vector3.right);
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
