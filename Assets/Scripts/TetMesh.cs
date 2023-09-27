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


    private void Awake()
    {
        Initialize();
        MeshManager.get.AddTetMesh(this);

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
    }
    void Start()
    {
        Debug.Log("TetMesh Start");
    }
    private void FixedUpdate()
    {
        DataRowMajor.ApplyDirty(state);
        DataRowMajor.ApplyDirtyToMesh(mesh);
        transform.position = DataRowMajor.com;
        // this.GetComponent<MeshCollider>().sharedMesh = mesh;
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
    //private void OnCollisionEnter(Collision collision)
    //{
    //    Debug.Log("OnCollisionEnter");
    //    int counts = collision.contactCount;
    //    Debug.Log("Collision count: " + counts);
    //    gameObject.GetComponent<Renderer>().material.SetColor("_Color", Color.red);

    //    for (int i = 0; i < counts; ++i)
    //    {
    //        var c = collision.GetContact(i);
    //        BackEnd.AddContact(c.point, c.normal, c.separation);
    //        Debug.Log("point: " + c.point + "\nnormal: " + c.normal + "\nseparation: " + c.separation);
    //    }
    //}
    //private void OnCollisionStay(Collision collision)
    //{
    //    Debug.Log("OnCollisionStay");
    //    int counts = collision.contactCount;
    //    Debug.Log("Collision count: " + counts);
    //    gameObject.GetComponent<Renderer>().material.SetColor("_Color", Color.blue);

    //    for (int i = 0; i < counts; ++i)
    //    {
    //        var c = collision.GetContact(i);
    //        BackEnd.AddContact(c.point, c.normal, c.separation);
    //        Debug.Log("point: " + c.point + "\nnormal: " + c.normal + "\nseparation: " + c.separation);
    //    }
    //}
    //private void OnCollisionExit(Collision collision)
    //{
        
    //}
}
