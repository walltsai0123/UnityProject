using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.Assertions;

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
        //BackEnd.CreateSoftBody();
        Debug.Log("MeshManager Awake");
    }
    private void Start()
    {
        //BackEnd.InitSoftBody();
        // BackEnd.AddPosConstraints(0, 1, Vector3.zero, Vector3.zero, 2f, 0f);
    }
    private void FixedUpdate()
    {
        BackEnd.XPBDSimUpdate(Time.fixedDeltaTime, 10);
        //BackEnd.SimulationUpdate(Time.fixedDeltaTime);
        Debug.Log("SimulationUpdate");
    }
    public unsafe int AddTetMesh(TetMesh tetMesh)
    {
        tetMeshes.Add(tetMesh);
        BackEnd.AddMesh(tetMesh.state, tetMesh.tetFileName,
            tetMesh.transform.position, tetMesh.transform.rotation,
            tetMesh.mass, tetMesh.mu, tetMesh.lambda, ((int)tetMesh.materialType));

        return tetMeshes.Count - 1;
    }
    private void OnDestroy()
    {
        //BackEnd.DeleteSoftBody();
        BackEnd.XPBDSimDelete();
        Debug.Log("MeshManager Destroy");
    }

}
