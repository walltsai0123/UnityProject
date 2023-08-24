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
    }
    private void FixedUpdate()
    {
        
    }
    public unsafe void AddTetMesh(TetMesh tetMesh)
    {
        tetMeshes.Add(tetMesh);
        BackEnd.AddMesh(tetMesh.state);
    }


}
