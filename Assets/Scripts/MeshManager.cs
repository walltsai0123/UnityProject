using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshManager : MonoBehaviour
{
    public static MeshManager get;
    public List<TetMesh> tetMeshes { get; private set; }

    private void Awake()
    {
        if(get)
        {
            Debug.LogWarning("MeshManager instance already exists.");
            enabled = false;
            return;
        }
        get = this;
        tetMeshes = new List<TetMesh>();
        BackEnd.CreateSoftBody();
        Debug.Log("MeshManager Awake");
    }
    private void Start()
    {
        BackEnd.InitSoftBody();
        Debug.Log("MeshManager Start");
    }
    private void FixedUpdate()
    {
        BackEnd.SimulationUpdate();
    }
    private unsafe void Update()
    {
        BackEnd.MeshesUpdate();
        foreach(var tm in tetMeshes)
        {
            tm.meshDirty = true;
        }
    }
    public unsafe void AddTetMesh(TetMesh tetMesh)
    {
        tetMeshes.Add(tetMesh);
        BackEnd.AddMesh(tetMesh.state);
    }
    private void OnDestroy()
    {
        BackEnd.DeleteSoftBody();
        Debug.Log("MeshManager Destroy");
    }

}
