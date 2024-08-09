using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using XPBD;
using UnityEngine.UIElements;
using UnityEngine.Assertions;
using System;
using UnityEngine.UI;
using UnityEngine.Rendering;
using Unity.VisualScripting;




#if USE_FLOAT
using REAL = System.Single;
using REAL2 = Unity.Mathematics.float2;
using REAL3 = Unity.Mathematics.float3;
using REAL4 = Unity.Mathematics.float4;
using REAL2x2 = Unity.Mathematics.float2x2;
using REAL3x3 = Unity.Mathematics.float3x3;
using REAL3x4 = Unity.Mathematics.float3x4;
#else
using REAL = System.Double;
using REAL2 = Unity.Mathematics.double2;
using REAL3 = Unity.Mathematics.double3;
using REAL4 = Unity.Mathematics.double4;
using REAL2x2 = Unity.Mathematics.double2x2;
using REAL3x3 = Unity.Mathematics.double3x3;
using REAL3x4 = Unity.Mathematics.double3x4;
#endif

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public class SoftBodySystem : MonoBehaviour
{
    private MeshRenderer meshRenderer;
    private MeshFilter meshFilter;
    private Mesh tetMesh;

    List<SoftBody> softBodies;
    int[] verticesOffset;
    int[] tetsOffset;
    int[] mu;
    int[] lambda;

    REAL3[] positions;
    REAL3[] velocities;
    REAL[] invMass;

    int4[] tets;
    REAL[] restVolumes;
    REAL3x3[] invDm;

    int[] indices;
    int[] passSize;


    public int VerticesNum { get; private set; }
    int TetsNum;

    // gpu kernels
    int preSolveKernel = 0;
    int solveElementKernel = 1;
    int postSolveKernel = 2;
    int updateMeshKernel = 3;

    public ComputeBuffer PositionBuffer => positionBuffer;
    public ComputeBuffer PrevPositionBuffer => prevPositionBuffer;
    public ComputeBuffer VelocityBuffer => velocityBuffer;
    public ComputeBuffer InverseMassBuffer => inverseMassBuffer;

    ComputeShader softbodyCS;
    [SerializeField] ComputeShader meshCS;
    ComputeBuffer positionBuffer;
    ComputeBuffer prevPositionBuffer;
    ComputeBuffer velocityBuffer;
    ComputeBuffer tetBuffer;
    ComputeBuffer inverseMassBuffer;
    ComputeBuffer invDmBuffer;
    ComputeBuffer restVolumeBuffer;
    ComputeBuffer elementsBuffer;
    GraphicsBuffer visMeshBuffer;
    GraphicsBuffer meshBuffer;
     
    public void OnDestroy()
    {
        ComputeHelper.Release(positionBuffer
            , prevPositionBuffer
            , velocityBuffer
            , inverseMassBuffer
            , tetBuffer
            , invDmBuffer
            , restVolumeBuffer
            , elementsBuffer);
    }
    public void CollectSoftBodies(List<SoftBody> bodies)
    {
        softBodies = bodies;
        // Calculate total vertices/tets number and offsets
        verticesOffset = new int[bodies.Count];
        tetsOffset = new int[bodies.Count];
        VerticesNum = 0;
        TetsNum = 0;
        for(int i = 0; i < bodies.Count; i++)
        {
            SoftBody b = bodies[i];
            
            int vNum = b.VerticesNum;
            int tNum = b.TetsNum;

            verticesOffset[i] = VerticesNum;
            tetsOffset[i] = TetsNum;

            VerticesNum += vNum;
            TetsNum += tNum;
        }

        positions = new REAL3[VerticesNum];
        velocities = new REAL3[VerticesNum];
        invMass = new REAL[VerticesNum];

        tets = new int4[TetsNum];
        restVolumes = new REAL[TetsNum];
        invDm = new REAL3x3[TetsNum];
        for(int i = 0; i < bodies.Count;i++)
        {
            SoftBody b = bodies[i];
            int startIndex = verticesOffset[i];
            for(int j = 0; j < b.VerticesNum; j++)
            {
                positions[startIndex + j] = b.Pos[j];
                velocities[startIndex + j] = b.Vel[j];
                invMass[startIndex + j] = b.invMass[j];
            }
            

            startIndex = tetsOffset[i];
            for (int j = 0; j < b.TetsNum; j++)
            {
                tets[startIndex + j] = b.tets[j] + verticesOffset[i];
                restVolumes[startIndex + j] = b.restVolumes[j];
                invDm[startIndex + j] = b.invDm[j];
            }
        }

        int[] colors = GraphColoring();
        int[] colorsize = new int[colors.Length];

        Assert.IsTrue(colors.Length == TetsNum);
        indices = new int[tets.Length];
        for (int i = 0; i < indices.Length; i++)
        {
            indices[i] = i;
        }

        // Sort indices in order of color
        Array.Sort(indices, (i1, i2) => colors[i1].CompareTo(colors[i2]));

        List<int> passSizeList = new List<int>();

        int currentColor = -1;
        for (int i = 0; i < indices.Length; i++)
        {
            int color = colors[indices[i]];

            if (currentColor != color)
            {
                currentColor = color;
                passSizeList.Add(0);
            }

            passSizeList[passSizeList.Count - 1]++;
        }
        passSize = passSizeList.ToArray();

        InitMesh();
        InitComputeBuffers();

        //softBodies[0].visualMesh.indexBufferTarget |= GraphicsBuffer.Target.Raw;

        //var indexbuffer = softBodies[0].visualMesh.GetIndexBuffer();
        //meshCS.SetBuffer(0, "indexbuffer", indexbuffer);

        //ComputeBuffer computeBuffer = ComputeHelper.CreateStructuredBuffer<uint>(indexbuffer.count);
        //meshCS.SetBuffer(0, "index", computeBuffer);

        //meshCS.SetInt("indexStride", indexbuffer.stride);
        //ComputeHelper.Dispatch(meshCS, indexbuffer.count, kernelIndex: 0);

        //uint[] datas = new uint[indexbuffer.count];
        //computeBuffer.GetData(datas);

        //UInt16[] data = new UInt16[indexbuffer.count];
        //indexbuffer.GetData(data);
        //for(int i = 0; i < datas.Length; i++)
        //{
        //    if (softBodies[0].visualMesh.triangles[i] != datas[i])
        //    {
        //        Debug.Log(i);

        //        Debug.Log("data: " + datas[i]);
        //    }
        //}

        //indexbuffer.Dispose();
        //computeBuffer.Dispose();
    }

    public void PreSolve(REAL dt, REAL3 gravity)
    {
        softbodyCS.SetFloat("dt", (float)dt);
        softbodyCS.SetInt("verticesNum", VerticesNum);
        softbodyCS.SetVector("gravity", new float4((float3)gravity, 0f));
        ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: preSolveKernel);
    }

    public void Solve(REAL dt)
    {
        softbodyCS.SetFloat("dt", (float)dt);

        softbodyCS.SetFloat("mu", 33557.05f);
        softbodyCS.SetFloat("lambda", 1644295);

        softbodyCS.SetFloat("mu", 10000);
        softbodyCS.SetFloat("lambda", 100000);
        int firstConstraint = 0;
        Timer timer = new();
        timer.Tic();
        foreach (int passNr in passSize)
        {
            softbodyCS.SetInt("passSize", passNr);
            softbodyCS.SetInt("firstConstraint", firstConstraint);
            ComputeHelper.Dispatch(softbodyCS, passNr, kernelIndex: solveElementKernel);

            firstConstraint += passNr;
        }

        //positionBuffer.GetData(new REAL3[VerticesNum]);
        //timer.Toc();
        //timer.Report(outputUnit : Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
    }
    public void PostSolve(REAL dt)
    {
        softbodyCS.SetFloat("dt", (float)dt);
        ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: postSolveKernel);
    }

    public void EndFrame()
    {
        for (int i = 0; i < softBodies.Count; i++)
        {
            SoftBody b = softBodies[i];
            //b.showTet = true;
            b.SetMesh();

            if(b.showTet)
            {
                Mesh mesh = b.tetMesh;
                mesh.bounds = new Bounds(Vector3.zero, 1000 * Vector3.one);
                mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
                meshBuffer = mesh.GetVertexBuffer(0);
                softbodyCS.SetBuffer(3, "meshVertices", meshBuffer);
                softbodyCS.SetInt("vertexStride", meshBuffer.stride);
                softbodyCS.SetInt("vertexOffset", verticesOffset[i]);
                softbodyCS.SetInt("verticesNum", mesh.vertexCount);
                ComputeHelper.Dispatch(softbodyCS, mesh.vertexCount, kernelIndex: 3);
            }
            else
            {
                Mesh mesh = b.visualMesh;
                mesh.bounds = new Bounds(Vector3.zero, 1000 * Vector3.one);
                mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
                meshBuffer = mesh.GetVertexBuffer(0);
                softbodyCS.SetBuffer(4, "meshVertices", meshBuffer);

                ComputeBuffer skinningBuffer = ComputeHelper.CreateStructuredBuffer<REAL4>(mesh.vertexCount);
                skinningBuffer.SetData(b.skinningInfo); 
                softbodyCS.SetBuffer(4, "skinningInfo", skinningBuffer);

                softbodyCS.SetInt("vertexStride", meshBuffer.stride);
                //softbodyCS.SetInt("vertexOffset", verticesOffset[i]);
                softbodyCS.SetInt("tetOffset", tetsOffset[i]);
                softbodyCS.SetInt("verticesNum", mesh.vertexCount);
                ComputeHelper.Dispatch(softbodyCS, mesh.vertexCount, kernelIndex: 4);

                skinningBuffer.Release();


            }
            meshBuffer.Dispose();
        }
    }

    private void InitMesh()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        // Create tetmesh from tetrahedron data
        tetMesh = new Mesh();
        tetMesh.name = gameObject.name + "_tetmesh";

        tetMesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;

        Vector3[] vertices = new Vector3[VerticesNum];
        for (int i = 0; i < VerticesNum; ++i)
            vertices[i] = (float3)positions[i];

        // Collect softbodies tetmesh faces
        List<int> tetMeshFaces = new List<int>();
        for(int i = 0; i < softBodies.Count; ++i)
        {
            SoftBody softBody = softBodies[i];

            int[] array = softBody.tetMesh.triangles;
            for (int j = 0; j < array.Length; j++)
            {
                array[j] += verticesOffset[i];
            }

            tetMeshFaces.AddRange(array);
        }

        tetMesh.SetVertices(vertices);
        tetMesh.SetIndices(tetMeshFaces, MeshTopology.Triangles, 0);
        tetMesh.RecalculateNormals();
        tetMesh.bounds = new Bounds(Vector3.zero, 1000 * Vector3.one);

        meshFilter.mesh = tetMesh;
        if(!meshRenderer.material)
            meshRenderer.material = new Material(Shader.Find("SuperSystems/Wireframe-Transparent-Culled"));
    }
    private void InitComputeBuffers()
    {
        // Load compute shader
        softbodyCS = Instantiate(Resources.Load<ComputeShader>("SoftBody"));

        positionBuffer = ComputeHelper.CreateStructuredBuffer<REAL3>(VerticesNum);
        prevPositionBuffer = ComputeHelper.CreateStructuredBuffer<REAL3>(VerticesNum);
        velocityBuffer = ComputeHelper.CreateStructuredBuffer<REAL3>(VerticesNum);
        inverseMassBuffer = ComputeHelper.CreateStructuredBuffer<REAL>(VerticesNum);
        tetBuffer = ComputeHelper.CreateStructuredBuffer<int4>(TetsNum);
        invDmBuffer = ComputeHelper.CreateStructuredBuffer<REAL3x3>(TetsNum);
        restVolumeBuffer = ComputeHelper.CreateStructuredBuffer<REAL>(TetsNum);
        elementsBuffer = ComputeHelper.CreateStructuredBuffer<int>(TetsNum);

        ComputeHelper.SetBuffer(softbodyCS, positionBuffer, "pos", preSolveKernel, solveElementKernel, postSolveKernel, 3, 4);
        ComputeHelper.SetBuffer(softbodyCS, prevPositionBuffer, "prevPos", preSolveKernel, postSolveKernel);
        ComputeHelper.SetBuffer(softbodyCS, velocityBuffer, "vel", preSolveKernel, solveElementKernel, postSolveKernel);
        ComputeHelper.SetBuffer(softbodyCS, tetBuffer, "tets", solveElementKernel, 4);
        ComputeHelper.SetBuffer(softbodyCS, inverseMassBuffer, "invMass", preSolveKernel, solveElementKernel, postSolveKernel);
        ComputeHelper.SetBuffer(softbodyCS, invDmBuffer, "invDm", solveElementKernel);
        ComputeHelper.SetBuffer(softbodyCS, restVolumeBuffer, "restVolumes", solveElementKernel);
        ComputeHelper.SetBuffer(softbodyCS, elementsBuffer, "elements", solveElementKernel);

        positionBuffer.SetData(positions);
        prevPositionBuffer.SetData(positions);
        velocityBuffer.SetData(velocities);
        inverseMassBuffer.SetData(invMass);
        tetBuffer.SetData(tets);
        invDmBuffer.SetData(invDm);
        restVolumeBuffer.SetData(restVolumes);
        elementsBuffer.SetData(indices);

        softbodyCS.SetInt("verticesNum", VerticesNum);
        softbodyCS.SetInt("tetsNum", TetsNum);
    }

    public int[] GraphColoring()
    {
        var adjacencyList = BuildAdjacencyList();

        int n = adjacencyList.Count;

        int[] colors = new int[n];
        bool[] available = new bool[n];

        for (int i = 0; i < n; i++)
            colors[i] = -1;

        colors[0] = 0;

        for (int i = 1; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                available[j] = true;

            foreach (int neighbor in adjacencyList[i])
            {
                if (colors[neighbor] != -1)
                    available[colors[neighbor]] = false;
            }

            int cr;
            for (cr = 0; cr < n; cr++)
            {
                if (available[cr])
                    break;
            }

            colors[i] = cr;
        }

        return colors;
    }

    private Dictionary<int, List<int>> BuildAdjacencyList()
    {
        Dictionary<int, List<int>> adjacencyList = new Dictionary<int, List<int>>();

        // traverse all tets
        for (int i = 0; i < tets.Length; i++)
        {
            //int4 tet1 = new(tets[4 * i], tets[4 * i + 1], tets[4 * i + 2], tets[4 * i + 3]);
            int4 tet1 = tets[i];
            for (int j = i + 1; j < tets.Length; j++)
            {
                //int4 tet2 = new(tets[4 * j], tets[4 * j + 1], tets[4 * j + 2], tets[4 * j + 3]);
                int4 tet2 = tets[j];
                // check adjacent
                if (AreAdjacent(tet1, tet2))
                {
                    if (!adjacencyList.ContainsKey(i))
                        adjacencyList[i] = new List<int>();

                    if (!adjacencyList.ContainsKey(j))
                        adjacencyList[j] = new List<int>();

                    // record adjacency
                    adjacencyList[i].Add(j);
                    adjacencyList[j].Add(i);
                }
            }
        }

        return adjacencyList;
    }

    private bool AreAdjacent(int4 t1, int4 t2)
    {
        int sharedVertices = 0;
        // check vertex equality
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (t1[i] == t2[j])
                    sharedVertices++;

                if (sharedVertices >= 1)
                    return true;
            }
        }
        return false;
    }
}
