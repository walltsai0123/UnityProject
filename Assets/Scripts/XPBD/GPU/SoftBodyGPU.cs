using UnityEngine;
using Unity.Mathematics;
using System;
using System.Collections.Generic;
using UnityEngine.Assertions;

//using System.Numerics;

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

namespace XPBD.GPU
{
    [RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public class SoftbodyGPU : Body
    {
        // Serialize fields
        [SerializeField] TetrahedronMesh tetrahedronMesh;
        [SerializeField] Material wireframeMaterial;
        [SerializeField] REAL mu = 10f, lambda = 1000f;
        [SerializeField] bool showTet = false;

        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        public Mesh visualMesh;
        private Mesh tetMesh;
        public Material visualMaterial { get; private set; }
        public Material collisionMaterial { get; private set; }
        public int VerticesNum { get; private set; }
        public int TetsNum { get; private set; }

        // GPU
        ComputeShader softbodyCS;
        ComputeBuffer positionBuffer;
        ComputeBuffer prevPositionBuffer;
        ComputeBuffer velocityBuffer;
        ComputeBuffer tetBuffer;
        ComputeBuffer inverseMassBuffer;
        ComputeBuffer invDmBuffer;
        ComputeBuffer restVolumeBuffer;
        ComputeBuffer elementsBuffer;
        ComputeBuffer skinningBuffer;
        GraphicsBuffer visMeshBuffer;
        GraphicsBuffer tetMeshBuffer;
        ComputeBuffer boundBuffer;
        int[] elementConstraitIds;
        int[] passSize;

        // gpu kernels
        int preSolveKernel = 0;
        int solveElementKernel = 1;
        int postSolveKernel = 2;

        #region Monobehaviour
        private void Awake()
        {
            InitializeMesh();
            InitializePhysics();

        }
        private void Start()
        {
            Simulation.get.AddBody(this);
            isStarted = true;
        }
        private void OnDestroy()
        {
            ComputeHelper.Release(
                positionBuffer, 
                prevPositionBuffer,
                velocityBuffer,
                inverseMassBuffer,
                tetBuffer,
                invDmBuffer,
                restVolumeBuffer,
                elementsBuffer,
                skinningBuffer,
                boundBuffer
                ); 
            tetMeshBuffer?.Release();
        }
        #endregion

        #region Body
        public override void PreSolve(REAL dt, REAL3 gravity)
        {
            REAL3 g = UseGravity ? gravity : REAL3.zero;

            softbodyCS.SetFloat("dt", (float)dt);
            softbodyCS.SetVector("gravity", new float4((float3)g, 0f));
            ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: preSolveKernel);

        }
        public override void Solve(REAL dt)
        {
            softbodyCS.SetFloat("mu", (float)mu);
            softbodyCS.SetFloat("lambda", (float)lambda);

            int firstConstraint = 0;

            foreach (int passNr in passSize)
            {
                softbodyCS.SetInt("passSize", passNr);
                softbodyCS.SetInt("firstConstraint", firstConstraint);
                ComputeHelper.Dispatch(softbodyCS, passNr, kernelIndex: solveElementKernel);

                firstConstraint += passNr;
            }
        }
        public override void PostSolve(REAL dt)
        {
            softbodyCS.SetFloat("dt", (float)dt);
            ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: postSolveKernel);
        }
        public override void EndFrame()
        {
            if (showTet || visualMesh == null)
            {
                meshFilter.mesh = tetMesh;
                meshRenderer.material = wireframeMaterial;

                tetMeshBuffer = tetMesh.GetVertexBuffer(0);

                softbodyCS.SetBuffer(3, "tetMeshVertices", tetMeshBuffer);
                softbodyCS.SetInt("vertexStride", tetMeshBuffer.stride);

                int groupsize = ComputeHelper.GetThreadGroupSizes(softbodyCS, 4).x;
                //ComputeBuffer groupSum = ComputeHelper.CreateStructuredBuffer<REAL3>(groupsize);

                //ComputeHelper.SetBuffer(softbodyCS, groupSum, "groupSum", 3, 4);


                ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: 3);

                //groupSum.Dispose();
                tetMeshBuffer.Dispose();
            }
        }
        #endregion
        #region IGrabbable
        public override void EndGrab(REAL3 grabPos, REAL3 vel)
        {
            return;
        }

        public override REAL3 GetGrabbedPos()
        {
            return 0;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            hit = null;
            return;
        }

        public override void MoveGrabbed(REAL3 grabPos)
        {
            return;
        }

        public override void StartGrab(REAL3 grabPos)
        {
            return;
        }

        
        #endregion

        private void InitializeMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();

            if (meshFilter.sharedMesh != null)
                visualMesh = meshFilter.mesh;

            // Create tetmesh from tetrahedron data
            tetMesh = new Mesh();
            tetMesh.name = gameObject.name + "_tetmesh";
            
            tetMesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            tetMesh.SetVertices(tetrahedronMesh.vertices);
            tetMesh.triangles = tetrahedronMesh.faces;
            tetMesh.RecalculateNormals();
            tetMesh.bounds = new Bounds(Vector3.zero, 1000 * Vector3.one);
            // Set materials
            visualMaterial = meshRenderer.material;
            collisionMaterial = new(Simulation.get.textureFrictionShader);

            if (showTet)
            {
                meshFilter.mesh = tetMesh; 
                meshRenderer.material = wireframeMaterial;
            }

        }
        private void InitializePhysics()
        {
            //bodyType = BodyType.Soft;
            VerticesNum = tetrahedronMesh.vertices.Length;
            TetsNum = tetrahedronMesh.tets.Length / 4;

            REAL3[] pos = new REAL3[VerticesNum];
            REAL3[] vel = new REAL3[VerticesNum];
            REAL[] invMass = new REAL[VerticesNum];

            int4[] tets = new int4[TetsNum]; 
            REAL[] restVolumes = new REAL[TetsNum];
            REAL3x3[] invDm = new REAL3x3[TetsNum];

            int EdgesNum = tetrahedronMesh.edges.Length / 2;
            REAL averageEdgeLength = 0;

            for (int i = 0; i < VerticesNum; i++)
            {
                pos[i] = (float3)tetrahedronMesh.vertices[i];
                vel[i] = 0;
                invMass[i] = 0;
            }

            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tetrahedronMesh.tets[4 * i + 0];
                int id1 = tetrahedronMesh.tets[4 * i + 1];
                int id2 = tetrahedronMesh.tets[4 * i + 2];
                int id3 = tetrahedronMesh.tets[4 * i + 3];

                tets[i] = new int4(id0, id1, id2, id3);
            }

            for (int i = 0; i < TetsNum; ++i)
            {
                restVolumes[i] = 0f;
                invDm[i] = REAL3x3.identity;
            }

            for (int i = 0; i < EdgesNum; ++i)
            {
                int id0 = tetrahedronMesh.edges[2 * i + 0];
                int id1 = tetrahedronMesh.edges[2 * i + 1];

                averageEdgeLength += math.length(pos[id0] - pos[id1]);
            }
            averageEdgeLength /= EdgesNum;

            // Rest volume
            REAL totalVolume = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;

                REAL3x3 RestPose = new(pos[id1] - pos[id0], pos[id2] - pos[id0], pos[id3] - pos[id0]);
                invDm[i] = math.inverse(RestPose);
                REAL V = math.determinant(RestPose) / 6f;

                REAL partialV = V / 4f;
                invMass[id0] += partialV;
                invMass[id1] += partialV;
                invMass[id2] += partialV;
                invMass[id3] += partialV;

                restVolumes[i] = V;
                totalVolume += V;
            }

            // Inverse mass (1/w)
            REAL density = mass / totalVolume;
            for (int i = 0; i < VerticesNum; ++i)
            {
                invMass[i] *= density;
                if (invMass[i] != 0f)
                    invMass[i] = 1f / invMass[i];
            }

            ComputeSkinningInfo(pos, tets, averageEdgeLength, out REAL4[] skinningInfo);

            //Translate to start position
            REAL3 origin = (float3)transform.position;
            transform.position = Vector3.zero;
            for(int i = 0; i < VerticesNum; i++)
            {
                pos[i] += origin;
            }

            // Do graph coloring to set up element constraint passes
            int[] colors = tetrahedronMesh.GraphColoring();
            int[] colorsize = new int[colors.Length];

            Assert.IsTrue(colors.Length == TetsNum);

            int[] indices = new int[tets.Length];
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

            Debug.Log("PassSize: " +  passSize.Length);
            foreach (int passNr in passSize)
                Debug.Log(passNr);

            //Init compute buffers
            InitializeComputeBuffers();

            positionBuffer.SetData(pos);
            prevPositionBuffer.SetData(pos);
            velocityBuffer.SetData(vel);
            inverseMassBuffer.SetData(invMass);
            tetBuffer.SetData(tets);
            invDmBuffer.SetData(invDm);
            restVolumeBuffer.SetData(restVolumes);
            elementsBuffer.SetData(indices);
        }

        private void ComputeSkinningInfo(REAL3[] pos, int4[] tets, REAL spacing, out REAL4[] skinningInfo)
        {
            skinningInfo = null;
            if (visualMesh == null)
                return;

            int numVisVerts = visualMesh.vertexCount;
            skinningInfo = new REAL4[numVisVerts];

            if (visualMesh == null)
                return;

            //float3[] visPos = new float3[numVisVerts];

            REAL3[] vertices = new REAL3[numVisVerts];
            for (int i = 0; i < numVisVerts; i++)
            {
                vertices[i] = (float3)visualMesh.vertices[i];
            }

            Hash hash = new Hash(spacing, numVisVerts);
            hash.Create(vertices);

            
            for (int i = 0; i < skinningInfo.Length; ++i)
                skinningInfo[i] = new REAL4(-1, -1, -1, -1);

            REAL[] minDist = new REAL[numVisVerts];
            Array.Fill(minDist, REAL.MaxValue);
            REAL border = 0.05f;

            // Each tet searches for containing vertices
            REAL3 tetCenter;
            REAL3x3 mat = REAL3x3.zero;
            REAL4 bary = REAL4.zero;
            //REAL averageEdge = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                // Compute bounding sphere for tet
                tetCenter = REAL3.zero;
                for (int j = 0; j < 4; ++j)
                    tetCenter += 0.25f * (pos[tets[i][j]]);

                REAL rMax = 0f;

                for (int j = 0; j < 4; ++j)
                {
                    REAL r2 = math.length(tetCenter - pos[tets[i][j]]);
                    rMax = math.max(rMax, r2);
                }

                rMax += border;

                hash.Query(tetCenter, rMax);
                if (hash.querySize == 0)
                    continue;

                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;

                mat.c0 = pos[id0] - pos[id3];
                mat.c1 = pos[id1] - pos[id3];
                mat.c2 = pos[id2] - pos[id3];

                mat = math.inverse(mat);

                for (int j = 0; j < hash.querySize; ++j)
                {
                    int id = hash.queryIds[j];

                    if (minDist[id] <= 0f)
                        continue;

                    REAL3 visVert = (float3)vertices[id];
                    if (math.lengthsq(visVert - tetCenter) > rMax * rMax)
                        continue;

                    bary.xyz = visVert - pos[id3];
                    bary.xyz = math.mul(mat, bary.xyz);
                    bary.w = 1f - bary.x - bary.y - bary.z;

                    REAL dist = 0f;

                    for (int k = 0; k < 4; ++k)
                        dist = math.max(dist, -bary[k]);

                    if (dist < minDist[id])
                    {
                        minDist[id] = dist;
                        skinningInfo[id] = new REAL4(i, bary[0], bary[1], bary[2]);
                    }
                }
            }
        }

        private void InitializeComputeBuffers()
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
            skinningBuffer = ComputeHelper.CreateStructuredBuffer<REAL4>(visualMesh.vertexCount);
            boundBuffer = ComputeHelper.CreateStructuredBuffer<REAL3>(3);

            ComputeHelper.SetBuffer(softbodyCS, positionBuffer, "pos", preSolveKernel, solveElementKernel, postSolveKernel, 3);
            ComputeHelper.SetBuffer(softbodyCS, prevPositionBuffer, "prevPos", preSolveKernel, postSolveKernel);
            ComputeHelper.SetBuffer(softbodyCS, velocityBuffer, "vel", preSolveKernel, solveElementKernel, postSolveKernel);
            ComputeHelper.SetBuffer(softbodyCS, tetBuffer, "tets", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, inverseMassBuffer, "invMass", preSolveKernel, solveElementKernel, postSolveKernel);
            ComputeHelper.SetBuffer(softbodyCS, invDmBuffer, "invDm", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, restVolumeBuffer, "restVolumes", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, elementsBuffer, "elements", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, boundBuffer, "bound", 3, 4);

            softbodyCS.SetInt("verticesNum", VerticesNum);
            softbodyCS.SetInt("tetsNum", TetsNum);

        }
    }
}

