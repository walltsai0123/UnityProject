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
    private Thread thread;
    private float startTime, endTime;

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
    }
    private void FixedUpdate()
    {
        //if (thread != null && !thread.IsAlive)
        //    PostExecuteThread();

        //if (thread == null)
        //    ExecuteThread();

        //startTime = Time.realtimeSinceStartup;
        BackEnd.SimulationUpdate();
        //endTime = Time.realtimeSinceStartup;
        //Debug.Log(endTime - startTime);
    }
    private void ExecuteThread()
    {
        Assert.IsTrue(thread == null);

        thread = new Thread(() => { BackEnd.SimulationUpdate(); } );
        thread.Name = "Mesh Manager Worker";

        startTime = Time.realtimeSinceStartup;
        thread.Start();
    }
    private void PostExecuteThread()
    {
        Assert.IsTrue(!thread.IsAlive);

        thread.Join();
        endTime = Time.realtimeSinceStartup;
        thread = null;

        Debug.Log(endTime - startTime);
    }
    private void Update()
    {
        BackEnd.MeshesUpdate();
        //foreach(var tm in tetMeshes)
        //{
        //    tm.meshDirty = true;
        //}
    }
    public unsafe void AddTetMesh(TetMesh tetMesh)
    {
        tetMeshes.Add(tetMesh);
        BackEnd.AddMesh(tetMesh.state, tetMesh.tetFileName);
    }
    private void OnDestroy()
    {
        if (thread != null)
        {
            thread.Join();
            thread = null;
        }
        BackEnd.DeleteSoftBody();
        Debug.Log("MeshManager Destroy");
    }

}
