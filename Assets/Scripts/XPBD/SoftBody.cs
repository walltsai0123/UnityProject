using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using System.Collections.Generic;
using System;
using UnityEngine.Assertions;
using System.Linq;

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

namespace XPBD
{
    
    public class SoftBody : Body
    {
        [SerializeField] TetrahedronMesh tetrahedronMesh;
        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        public Mesh visualMesh;
        public Mesh tetMesh;
        public Material visualMaterial { get; private set; }
        public Material collisionMaterial { get; private set; }
        [SerializeField] Material wireframeMaterial;

        public REAL mu = 10f, lambda = 1000f;
        public bool showTet = false;

        // Simulation Data NativeArray
        public NativeArray<REAL3> Pos;
        public NativeArray<REAL3> prevPos;
        public NativeArray<REAL3> Vel;
        public NativeArray<int4> tets;
        public NativeArray<REAL> invMass;
        public NativeArray<REAL> restVolumes;
        public NativeArray<REAL3x3> invDm;
        private NativeArray<int2> edges;
        private NativeArray<REAL> restEdgeLengths;

        private NativeArray<int3> volIdOrder;
        public REAL3 fext { get; set; } = REAL3.zero;

        // Skinning Info
        private NativeArray<Vector3> visPos;
        public NativeArray<REAL4> skinningInfo;

        public int VerticesNum { get; private set; }
        public int TetsNum { get; private set; }

        private int EdgesNum;

        [SerializeField]
        private REAL3 v0 = REAL3.zero;
        [SerializeField]
        private REAL3 translate = REAL3.zero;

        private JobHandle jobHandle;
        //private ComputeShader computeShader = null;

        public List<MyCollision> collisions;

        // Grabber info
        public int grabId { get; private set; }
        private REAL grabInvMass;

        // GPU
        ComputeShader softbodyCS;
        ComputeBuffer positionBuffer;
        ComputeBuffer prevPositionBuffer;
        ComputeBuffer correctionBuffer;
        ComputeBuffer velocityBuffer;
        ComputeBuffer tetBuffer;
        ComputeBuffer inverseMassBuffer;
        ComputeBuffer invDmBuffer;
        ComputeBuffer restVolumeBuffer;
        ComputeBuffer elementsBuffer;
        int[] elementConstraitIds;
        int[] passSize;
        // gpu kernels
        int preSolveKernel = 0;
        int solveElementKernel = 1;

        REAL3 startPos;
        #region Body
        public override void ClearCollision()
        {
            collisions.Clear();

            collisionMaterial.CopyMatchingPropertiesFromMaterial(visualMaterial);
        }

        public override void PreSolve(REAL dt, REAL3 gravity)
        {
            REAL3 g = REAL3.zero;
            if (UseGravity)
                g = gravity;

            g += fext;

            PreSolveJob preSolveJob = new()
            {
                dt = dt,
                gravity = g,
                pos = Pos,
                prevPos = prevPos,
                vel = Vel,
                invMass = invMass
            };
            jobHandle = preSolveJob.Schedule(VerticesNum, 1, jobHandle);
            jobHandle.Complete();

        }

        public override void Solve(REAL dt)
        {
            if(Simulation.get.UseNeoHookeanMaterial)
            {
                SolveElementJob solveElementJob = new SolveElementJob
                {
                    dt = dt,
                    mu = this.mu,
                    lambda = this.lambda,
                    pos = this.Pos,
                    tets = this.tets,
                    restVolumes = this.restVolumes,
                    invDm = this.invDm,
                    invMass = this.invMass
                };
                jobHandle = solveElementJob.Schedule(jobHandle);
                jobHandle.Complete();

            }
            else
            {
                SolveElementJob2 solveElementJob2 = new SolveElementJob2
                {
                    dt = dt,
                    mu = this.mu,
                    lambda = this.lambda,
                    pos = this.Pos,
                    edges = this.edges,
                    tets = this.tets,
                    restEdgeLengths = this.restEdgeLengths,
                    restVolumes = this.restVolumes,
                    invMass = this.invMass,
                    volIdOrder = this.volIdOrder
                };
                jobHandle = solveElementJob2.Schedule(jobHandle);
                jobHandle.Complete();
            }
        }

        public override void PostSolve(REAL dt)
        {
            for (int i = 0; i < VerticesNum; ++i)
            {
                // Ignore zero mass
                if (invMass[i] == 0)
                    continue;

                if (!EnableContact && Pos[i].y < 0f)
                    Pos[i] -= new REAL3(0, Pos[i].y, 0);

                Vel[i] = (Pos[i] - prevPos[i]) / dt;
            }
        }

        public override void EndFrame()
        {
            if (showTet || visualMesh == null)
            {
                meshFilter.mesh = tetMesh;
                meshRenderer.material = wireframeMaterial;

                Vector3[] V = new Vector3[VerticesNum];
                for (int i = 0; i < VerticesNum; i++)
                    V[i] = (float3)Pos[i];
                tetMesh.SetVertices(V);
                tetMesh.RecalculateBounds();
                tetMesh.RecalculateNormals();
                tetMesh.RecalculateTangents();
            }
            else
            {
                meshFilter.mesh = visualMesh;
                meshRenderer.material = visualMaterial;

                UpdateVisMeshJob updateVisMeshJob = new UpdateVisMeshJob
                {
                    visPos = visPos,
                    pos = Pos,
                    tets = tets,
                    skinningInfo = skinningInfo
                };
                jobHandle = updateVisMeshJob.Schedule(visPos.Length, 1, jobHandle);
                jobHandle.Complete();

                visualMesh.SetVertices(visPos);
                visualMesh.RecalculateBounds();
                visualMesh.RecalculateNormals();
                visualMesh.RecalculateTangents();
            }
        }
        
        public void SetMesh()
        {
            if(showTet)
            {
                meshFilter.mesh = tetMesh;
                meshRenderer.material = wireframeMaterial;
            }
            else
            {
                meshFilter.mesh = visualMesh;
                meshRenderer.material = visualMaterial;
            }
        }
        #endregion

        #region IGrabbable
        public override void StartGrab(REAL3 grabPos)
        {
            //Find the closest vertex to the pos on a triangle in the mesh
            REAL minD2 = REAL.MaxValue;
            REAL3 gPos = grabPos;
            grabId = -1;

            for (int i = 0; i < VerticesNum; i++)
            {

                REAL d2 = math.lengthsq(gPos - Pos[i]);

                if (d2 < minD2)
                {
                    minD2 = d2;
                    grabId = i;
                }
            }

            //We have found a vertex
            if (grabId >= 0)
            {
                //Save the current innverted mass
                grabInvMass = invMass[grabId];

                //Set the inverted mass to 0 to mark it as fixed
                invMass[grabId] = 0;

                //Set the position of the vertex to the position where the ray hit the triangle
                Pos[grabId] = grabPos;
            }
        }

        public override void MoveGrabbed(REAL3 grabPos)
        {
            if (grabId >= 0)
            {
                Pos[grabId] = grabPos;
            }
        }

        public override void EndGrab(REAL3 grabPos, REAL3 vel)
        {
            if (grabId >= 0)
            {
                //Set the mass to whatever mass it was before we grabbed it
                invMass[grabId] = grabInvMass;

                this.Vel[grabId] = 0;
            }

            grabId = -1;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            REAL3[] vertices = Pos.ToArray();
            int[] triangles = tetrahedronMesh.faces;

            Intersection.IsRayHittingMesh(ray, vertices, triangles, out hit);
        }

        public override REAL3 GetGrabbedPos()
        {
            return (float3)Pos[grabId];
        }
        #endregion

        #region Monobehaviour
        private void Awake()
        {
            Assert.IsNotNull(tetrahedronMesh);

            InitializeMesh();

            // Move to starting position
            startPos = (float3)transform.position;
            transform.position = Vector3.zero;
        }
        private void OnEnable()
        {
            InitializePhysics2();
            InitializeComputeBuffers();
            Simulation.get.AddBody(this);
        }
        private void Start()
        {
            isStarted = true;
        }
        private void OnDrawGizmosSelected()
        {
            if (!isStarted)
                return;

            //for(int i = 0; i < VerticesNum; i++)
            //{
            //    DrawArrow.ForGizmo((float3)Pos[i], (float3)Vel[i], Color.red);
            //}
            REAL3 Q = 0;
            REAL3 F = 0;
            foreach (MyCollision collision in collisions)
            {

                REAL3 FT = collision.ft / 60.0f;
                REAL3 FB = collision.fb / 60.0f;

                Q += collision.q;
                F += FB + FT;

                //DrawArrow.ForGizmo((float3)collision.q, (float3)FT, Color.red);
                //DrawArrow.ForGizmo((float3)collision.q, (float3)FB, Color.blue);
                //DrawArrow.ForGizmo((float3)collision.q, (float3)F, Color.green);
            }
            DrawArrow.ForGizmo((float3)Q / collisions.Count, (float3)F, Color.green);
        }

        private void OnDisable()
        {
            if (Pos.IsCreated) Pos.Dispose();
            if (prevPos.IsCreated) prevPos.Dispose();
            if (Vel.IsCreated) Vel.Dispose();
            if (invMass.IsCreated) invMass.Dispose();
            if (tets.IsCreated) tets.Dispose();
            if (restVolumes.IsCreated) restVolumes.Dispose();
            if (invDm.IsCreated) invDm.Dispose();
            if (visPos.IsCreated) visPos.Dispose();
            if (skinningInfo.IsCreated) skinningInfo.Dispose();

            if (edges.IsCreated) edges.Dispose();
            if (restEdgeLengths.IsCreated) restEdgeLengths.Dispose();
            if (volIdOrder.IsCreated) volIdOrder.Dispose();

            ComputeHelper.Release(positionBuffer, prevPositionBuffer, velocityBuffer, correctionBuffer,
                tetBuffer, inverseMassBuffer, invDmBuffer, restVolumeBuffer, elementsBuffer);

            Simulation.get.RemoveBody(this);
        }
        #endregion

        private void InitializeMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();

            if(meshFilter.sharedMesh != null)
            {
                //Mesh mesh = meshFilter.mesh;
                //visualMesh = new Mesh();
                //visualMesh.name = mesh.name;
                //visualMesh.SetVertices(mesh.vertices);
                //visualMesh.SetIndices(mesh.triangles, MeshTopology.Triangles, 0);
                //visualMesh.RecalculateNormals();
                //visualMesh.RecalculateTangents();
                //meshFilter.mesh = visualMesh;
                visualMesh = meshFilter.mesh;
            }
                

            // Create tetmesh from tetrahedron data
            tetMesh = new Mesh();
            tetMesh.name = gameObject.name + "_tetmesh";
            tetMesh.SetVertices(tetrahedronMesh.vertices);
            tetMesh.SetIndices(tetrahedronMesh.faces, MeshTopology.Triangles, 0);
            tetMesh.RecalculateNormals();

            // Set materials
            visualMaterial = meshRenderer.material;
            collisionMaterial = new(Simulation.get.textureFrictionShader);

        }
        private void InitializePhysics2()
        {
            bodyType = BodyType.Soft;

            VerticesNum = tetrahedronMesh.vertices.Length;
            TetsNum = tetrahedronMesh.tets.Length / 4;
            EdgesNum = tetrahedronMesh.edges.Length / 2;

            Pos = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            prevPos = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            Vel = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            invMass = new NativeArray<REAL>(VerticesNum, Allocator.Persistent);

            tets = new NativeArray<int4>(TetsNum, Allocator.Persistent);
            restVolumes = new NativeArray<REAL>(TetsNum, Allocator.Persistent);
            invDm = new NativeArray<REAL3x3>(TetsNum, Allocator.Persistent);

            edges = new NativeArray<int2>(EdgesNum, Allocator.Persistent);
            restEdgeLengths = new NativeArray<REAL>(EdgesNum, Allocator.Persistent);

            volIdOrder = new NativeArray<int3>(4, Allocator.Persistent);
            volIdOrder[0] = new(1, 3, 2);
            volIdOrder[1] = new(0, 2, 3);
            volIdOrder[2] = new(0, 3, 1);
            volIdOrder[3] = new(0, 1, 2);

            // Initialize NativeArrays
            for (int i = 0; i < VerticesNum; ++i)
            {
                Pos[i] = prevPos[i] = (float3)tetrahedronMesh.vertices[i] + startPos;
                Vel[i] = v0;
                invMass[i] = 0f;
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
            for(int i = 0; i < EdgesNum; ++i)
            {
                int id0 = tetrahedronMesh.edges[2 * i + 0];
                int id1 = tetrahedronMesh.edges[2 * i + 1];

                edges[i] = new int2(id0, id1);
                restEdgeLengths[i] = math.length(Pos[id0] - Pos[id1]);
            }

            // Rest volume
            REAL totalVolume = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;


                REAL3x3 RestPose = new (Pos[id1] - Pos[id0], Pos[id2] - Pos[id0], Pos[id3] - Pos[id0]);
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

            //Debug.Log("Total volume: " + totalVolume);
            //Debug.Log("Density: " + density);

            // Visual embedded mesh
            ComputeSkinningInfo2(startPos);

            // Init collision list
            collisions = new List<MyCollision>();

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
            for(int i = 0; i < indices.Length; i++)
            {
                int color = colors[indices[i]];

                if(currentColor != color)
                {
                    currentColor = color;
                    passSizeList.Add(0);
                }

                passSizeList[passSizeList.Count - 1]++;
            }

            elementConstraitIds = indices.ToArray();
            passSize = passSizeList.ToArray();

        }

        private void ComputeSkinningInfo2(REAL3 startPos)
        {
            if(visualMesh == null)
                return;
            int numVisVerts = visualMesh.vertexCount;
            visPos = new NativeArray<Vector3>(visualMesh.vertexCount, Allocator.Persistent);

            REAL3[] vertices = new REAL3[numVisVerts];
            for (int i = 0; i < numVisVerts; i++)
            {
                vertices[i] = (float3)visualMesh.vertices[i] + startPos;
            }

            Hash hash = new Hash(0.15f, numVisVerts);
            hash.Create(vertices);

            skinningInfo = new NativeArray<REAL4>(numVisVerts, Allocator.Persistent);
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
                    tetCenter += 0.25f * (Pos[tets[i][j]]);

                REAL rMax = 0f;

                for (int j = 0; j < 4; ++j)
                {
                    REAL r2 = math.length(tetCenter - Pos[tets[i][j]]);
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

                mat.c0 = Pos[id0] - Pos[id3];
                mat.c1 = Pos[id1] - Pos[id3];
                mat.c2 = Pos[id2] - Pos[id3];

                mat = math.inverse(mat);

                for (int j = 0; j < hash.querySize; ++j)
                {
                    int id = hash.queryIds[j];

                    if (minDist[id] <= 0f)
                        continue;

                    REAL3 visVert = (float3)vertices[id];
                    if (math.lengthsq(visVert - tetCenter) > rMax * rMax)
                        continue;

                    bary.xyz = visVert - Pos[id3];
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

            positionBuffer = ComputeHelper.CreateStructuredBuffer<float3>(VerticesNum);
            prevPositionBuffer = ComputeHelper.CreateStructuredBuffer<float3>(VerticesNum);
            correctionBuffer = ComputeHelper.CreateStructuredBuffer<float3>(VerticesNum);
            velocityBuffer = ComputeHelper.CreateStructuredBuffer<float3>(VerticesNum);
            tetBuffer = ComputeHelper.CreateStructuredBuffer<int4>(TetsNum);
            inverseMassBuffer = ComputeHelper.CreateStructuredBuffer<float>(VerticesNum);
            invDmBuffer = ComputeHelper.CreateStructuredBuffer<float3x3>(TetsNum);
            restVolumeBuffer = ComputeHelper.CreateStructuredBuffer<float>(TetsNum);
            elementsBuffer = ComputeHelper.CreateStructuredBuffer<int>(TetsNum);

            // TODO: bind buffer to compute shader kernel
            ComputeHelper.SetBuffer(softbodyCS, positionBuffer, "pos", preSolveKernel, solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, prevPositionBuffer, "prevPos", preSolveKernel);
            ComputeHelper.SetBuffer(softbodyCS, correctionBuffer, "corr", preSolveKernel, solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, velocityBuffer, "vel", preSolveKernel, solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, tetBuffer, "tets",solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, inverseMassBuffer, "invMass", preSolveKernel, solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, invDmBuffer, "invDm", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, restVolumeBuffer, "restVolumes", solveElementKernel);
            ComputeHelper.SetBuffer(softbodyCS, elementsBuffer, "elements", solveElementKernel);

            //tetBuffer.SetData(tets);
            //invDmBuffer.SetData(invDm);
            //restVolumeBuffer.SetData(restVolumes);
            //inverseMassBuffer.SetData(invMass);
            //elementsBuffer.SetData(elementConstraitIds);

            softbodyCS.SetInt("verticesNum", VerticesNum);
            softbodyCS.SetInt("tetsNum", TetsNum);
        }

        private void PreSolveGPU(float dt, float3 gravity)
        {
            softbodyCS.SetFloat("dt", dt);
            softbodyCS.SetVector("gravity", new float4(gravity, 0f));

            positionBuffer.SetData(Pos);
            velocityBuffer.SetData(Vel);

            ComputeHelper.Dispatch(softbodyCS, VerticesNum, kernelIndex: preSolveKernel);
        }
        private void SolveElementGPU(float dt)
        {
            softbodyCS.SetFloat("dt", dt);
            softbodyCS.SetFloat("mu", (float)mu);
            softbodyCS.SetFloat("lambda", (float)lambda);

            int firstConstraint = 0;

            foreach(int passNr in passSize)
            {
                softbodyCS.SetInt("passSize", passNr);
                softbodyCS.SetInt("firstConstraint", firstConstraint);
                ComputeHelper.Dispatch(softbodyCS, passNr, kernelIndex: solveElementKernel);

                firstConstraint += passNr;
            }
        }

        private REAL GetEnergy()
        {
            REAL energy_D = 0f;
            REAL energy_V = 0f;
            REAL energy = 0f;
            REAL gamma = mu / lambda;

            REAL3x3 PK1 = REAL3x3.zero;
            for (int i = 0; i < TetsNum; i++)
            {
                REAL3x3 F = GetDeformationGradient(i);
                REAL J = math.determinant(F);
                REAL energyD = 0.5f * mu / (math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2) - 3f);
                REAL energyV = 0.5f * lambda / math.pow(J - 1f - gamma, 2f);
                energy_D += energyD * restVolumes[i];
                energy_V += energyV * restVolumes[i];
                energy += (energyD + energyV) * restVolumes[i];

                REAL3x3 PJPF = new REAL3x3(math.cross(F.c1, F.c2), math.cross(F.c2, F.c0), math.cross(F.c0, F.c1));
                PK1 += mu * F + PJPF * (lambda * (J - 1f) - mu);
            }

            Debug.Log("PK1: " + PK1);
            Debug.Log("energyD: " + energy_D);
            Debug.Log("energyV: " + energy_V);
            Debug.Log("energy: " + energy);

            return energy;
        }

        private void Translate(REAL x, REAL y, REAL z)
        {
            for(int i = 0; i < VerticesNum; i++)
            {
                Pos[i] += new REAL3(x, y, z);
                prevPos[i] = Pos[i];
            }
        }

        private REAL3x3 GetDeformationGradient(int tetIndex)
        {
            int id0 = tets[tetIndex].x;
            int id1 = tets[tetIndex].y;
            int id2 = tets[tetIndex].z;
            int id3 = tets[tetIndex].w;

            REAL3x3 Ds = REAL3x3.zero;
            Ds.c0 = Pos[id1] - Pos[id0];
            Ds.c1 = Pos[id2] - Pos[id0];
            Ds.c2 = Pos[id3] - Pos[id0];

            return math.mul(Ds, invDm[tetIndex]);
        }

        #region IJob
        [BurstCompile]
        private struct PreSolveJob : IJobParallelFor
        {
            public REAL dt;
            public REAL3 gravity;

            public NativeArray<REAL3> pos;
            public NativeArray<REAL3> prevPos;
            public NativeArray<REAL3> vel;

            [ReadOnly]
            public NativeArray<REAL> invMass;

            public void Execute(int index)
            {
                if (invMass[index] == 0f)
                    return;

                vel[index] += dt * gravity;

                prevPos[index] = pos[index];

                pos[index] += dt * vel[index];
            }
        }

        [BurstCompile]
        private struct SolveElementJob : IJob
        {
            public REAL dt;
            public REAL mu;
            public REAL lambda;

            public NativeArray<REAL3> pos;

            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<REAL> restVolumes;
            [ReadOnly]
            public NativeArray<REAL> invMass;
            [ReadOnly]
            public NativeArray<REAL3x3> invDm;

            private REAL3x4 gradients;
            private REAL3x4 gradients_d;
            private REAL3x3 F, dF;
            public void Execute()
            {
                for (int index = 0; index < tets.Length; ++index)
                {
                    //SolveDeviatoric(index);
                    //SolveVolumetric(index);
                    SolveCoupled(index);
                }
            }

            private REAL3x3 GetDeformationGradient(int tetIndex)
            {
                int id0 = tets[tetIndex].x;
                int id1 = tets[tetIndex].y;
                int id2 = tets[tetIndex].z;
                int id3 = tets[tetIndex].w;

                REAL3x3 Ds = REAL3x3.zero;
                Ds.c0 = pos[id1] - pos[id0];
                Ds.c1 = pos[id2] - pos[id0];
                Ds.c2 = pos[id3] - pos[id0];
                
                return math.mul(Ds, invDm[tetIndex]);
            }
            private void SolveDeviatoric(int i)
            {
                REAL compliance = 1f / (mu * restVolumes[i]);
                REAL3x3 F = GetDeformationGradient(i);
                REAL r_s = math.sqrt(math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2));
                REAL C = r_s;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL r_s_inv = 1f / r_s;

                REAL3x3 QT = math.transpose(invDm[i]);
                REAL3x3 dC_D = r_s_inv * math.mul(F, QT);

                REAL3 grad1_D = dC_D.c0;
                REAL3 grad2_D = dC_D.c1;
                REAL3 grad3_D = dC_D.c2;
                REAL3 grad0_D = -grad1_D - grad2_D - grad3_D;

                // gradients set zero
                gradients_d = REAL3x4.zero;

                gradients_d = REAL3x4.zero;
                gradients_d.c0 = grad0_D;
                gradients_d.c1 = grad1_D;
                gradients_d.c2 = grad2_D;
                gradients_d.c3 = grad3_D;

                ApplyToElement(i, C, compliance, dt);
            }

            private void SolveVolumetric(int i)
            {
                REAL gamma = 1f + mu / lambda;
                REAL compliance = 1f / (lambda * restVolumes[i]);

                REAL3x3 F = GetDeformationGradient(i);
                REAL C = math.determinant(F) - gamma;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL3x3 QT = math.transpose(invDm[i]);

                REAL3 f1 = F.c0;
                REAL3 f2 = F.c1;
                REAL3 f3 = F.c2;

                REAL3x3 d_F = new REAL3x3(math.cross(f2, f3), math.cross(f3, f1), math.cross(f1, f2));
                REAL3x3 dC_V = math.mul(d_F, QT);

                REAL3 grad1_V = dC_V.c0;
                REAL3 grad2_V = dC_V.c1;
                REAL3 grad3_V = dC_V.c2;
                REAL3 grad0_V = -grad1_V - grad2_V - grad3_V;

                REAL3x3 dF = REAL3x3.zero;
                dF.c0 = math.cross(F.c1, F.c2);
                dF.c1 = math.cross(F.c2, F.c0);
                dF.c2 = math.cross(F.c0, F.c1);

                // gradients set zero
                gradients_d = REAL3x4.zero;
                gradients_d.c0 = grad0_V;
                gradients_d.c1 = grad1_V;
                gradients_d.c2 = grad2_V;
                gradients_d.c3 = grad3_V;

                ApplyToElement(i, C, compliance, dt);

            }

            private void SolveCoupled(int i)
            {
                // vertices index
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;

                REAL compliance_D = 1f / (mu * restVolumes[i]);

                REAL gamma = mu / lambda;
                REAL compliance_V = 1f / (lambda * restVolumes[i]);

                REAL3x3 QT = math.transpose(invDm[i]);

                REAL3x3 F = GetDeformationGradient(i);
                REAL3 f1 = F.c0;
                REAL3 f2 = F.c1;
                REAL3 f3 = F.c2;

                REAL r_s = math.sqrt(math.dot(f1,f1) + math.dot(f2, f2) + math.dot(f3, f3));
                REAL r_s_inv = 1f / r_s;

                REAL C_D = r_s;
                REAL C_V = math.determinant(F) - 1f - gamma;

                if (math.abs(C_D) >= Util.EPSILON && math.abs(C_V) >= Util.EPSILON)
                {
                    // compliance matrix
                    REAL2x2 alpha = new REAL2x2(compliance_D, 0, 0, compliance_V);
                    alpha /= (dt * dt);

                    REAL3x3 dC_D = r_s_inv * math.mul(F, QT);

                    REAL3 grad1_D = dC_D.c0;
                    REAL3 grad2_D = dC_D.c1;
                    REAL3 grad3_D = dC_D.c2;
                    REAL3 grad0_D = -grad1_D - grad2_D - grad3_D;

                    REAL3x3 d_F = new REAL3x3(math.cross(f2, f3), math.cross(f3, f1), math.cross(f1, f2));
                    REAL3x3 dC_V = math.mul(d_F, QT);

                    REAL3 grad1_V = dC_V.c0;
                    REAL3 grad2_V = dC_V.c1;
                    REAL3 grad3_V = dC_V.c2;
                    REAL3 grad0_V = -grad1_V - grad2_V - grad3_V;

                    REAL2x2 A = REAL2x2.zero;
                    A[0][0] += math.dot(grad0_D, grad0_D) * invMass[id0];
                    A[0][0] += math.dot(grad1_D, grad1_D) * invMass[id1];
                    A[0][0] += math.dot(grad2_D, grad2_D) * invMass[id2];
                    A[0][0] += math.dot(grad3_D, grad3_D) * invMass[id3];

                    A[1][0] += math.dot(grad0_D, grad0_V) * invMass[id0];
                    A[1][0] += math.dot(grad1_D, grad1_V) * invMass[id1];
                    A[1][0] += math.dot(grad2_D, grad2_V) * invMass[id2];
                    A[1][0] += math.dot(grad3_D, grad3_V) * invMass[id3];

                    A[0][1] = A[1][0];

                    A[1][1] += math.dot(grad0_V, grad0_V) * invMass[id0];
                    A[1][1] += math.dot(grad1_V, grad1_V) * invMass[id1];
                    A[1][1] += math.dot(grad2_V, grad2_V) * invMass[id2];
                    A[1][1] += math.dot(grad3_V, grad3_V) * invMass[id3];

                    A += alpha;

                    REAL2 dlambda = Util.LUSolve(A, new REAL2(-C_D, -C_V));

                    pos[id0] = (REAL3)(pos[id0] + dlambda[0] * invMass[id0] * grad0_D + dlambda[1] * invMass[id0] * grad0_V);
                    pos[id1] = (REAL3)(pos[id1] + dlambda[0] * invMass[id1] * grad1_D + dlambda[1] * invMass[id1] * grad1_V);
                    pos[id2] = (REAL3)(pos[id2] + dlambda[0] * invMass[id2] * grad2_D + dlambda[1] * invMass[id2] * grad2_V);
                    pos[id3] = (REAL3)(pos[id3] + dlambda[0] * invMass[id3] * grad3_D + dlambda[1] * invMass[id3] * grad3_V);
                }
                //else
                //{
                //    SolveDeviatoric(i);
                //    SolveVolumetric(i);
                //}

            }
            private void ApplyToElement(int i, REAL C, REAL compliance, REAL dt)
            {
                REAL weight = 0f;
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;
                weight += math.lengthsq(gradients_d.c0) * invMass[id0];
                weight += math.lengthsq(gradients_d.c1) * invMass[id1];
                weight += math.lengthsq(gradients_d.c2) * invMass[id2];
                weight += math.lengthsq(gradients_d.c3) * invMass[id3];

                if (weight < Util.EPSILON)
                    return;

                REAL h2 = dt * dt;
                REAL alpha = compliance / h2;
                REAL dlambda = -C / (weight + alpha);

                pos[id0] = (REAL3)(pos[id0] + dlambda * invMass[id0] * gradients_d.c0);
                pos[id1] = (REAL3)(pos[id1] + dlambda * invMass[id1] * gradients_d.c1);
                pos[id2] = (REAL3)(pos[id2] + dlambda * invMass[id2] * gradients_d.c2);
                pos[id3] = (REAL3)(pos[id3] + dlambda * invMass[id3] * gradients_d.c3);
            }
        }

        [BurstCompile]
        private struct SolveElementJob2 : IJob
        {
            public REAL dt;
            public REAL mu;
            public REAL lambda;

            public NativeArray<REAL3> pos;

            [ReadOnly]
            public NativeArray<int2> edges;
            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<REAL> restEdgeLengths;
            [ReadOnly]
            public NativeArray<REAL> restVolumes;
            [ReadOnly]
            public NativeArray<REAL> invMass;

            private REAL3x4 gradients;

            public NativeArray<int3> volIdOrder;

            public void Execute()
            {
                SolveEdges();
                SolveVolumes();
            }

            private void SolveEdges()
            {
                REAL compliance = 1f / mu;
                REAL alpha = compliance / (dt * dt);

                //For each edge
                for (int i = 0; i < edges.Length; ++i)
                {
                    //2 vertices per edge in the data structure, so multiply by 2 to get the correct vertex index
                    int id0 = edges[i].x;
                    int id1 = edges[i].y;

                    REAL w0 = invMass[id0];
                    REAL w1 = invMass[id1];

                    REAL wTot = w0 + w1;

                    //This edge is fixed so dont simulate
                    if (wTot < Util.EPSILON)
                        continue;

                    //x0-x1
                    //The result is stored in grads array
                    REAL3 id0_minus_id1 = pos[id0] - pos[id1];

                    //sqrMargnitude(x0-x1)
                    REAL l = math.length(id0_minus_id1);

                    //If they are at the same pos we get a divisio by 0 later so ignore
                    if (l < Util.EPSILON)
                    {
                        continue;
                    }

                    //(x0-x1) * (1/|x0-x1|) = gradC
                    REAL3 gradC = id0_minus_id1 / l;

                    REAL l_rest = restEdgeLengths[i];

                    REAL C = l - l_rest;

                    //lambda because |grad_Cn|^2 = 1 because if we move a particle 1 unit, the distance between the particles also grows with 1 unit, and w = w0 + w1
                    REAL lambda = -C / (wTot + alpha);

                    //Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
                    pos[id0] += lambda * w0 * gradC;
                    pos[id1] += -lambda * w1 * gradC;
                }
            }

            private void SolveVolumes()
            {
                REAL compliance = 1f / lambda;
                REAL alpha = compliance / (dt * dt);

                for (int i = 0; i < tets.Length; ++i)
                {
                    REAL wTimesGrad = 0f;

                    //Foreach vertex in the tetra
                    for (int j = 0; j < 4; j++)
                    {
                        int idThis = tets[i][j];

                        //The 3 opposite vertices ids
                        int id0 = tets[i][volIdOrder[j][0]];
                        int id1 = tets[i][volIdOrder[j][1]];
                        int id2 = tets[i][volIdOrder[j][2]];

                        //(x4 - x2)
                        REAL3 id1_minus_id0 = pos[id1] - pos[id0];
                        //(x3 - x2)
                        REAL3 id2_minus_id0 = pos[id2] - pos[id0];

                        //(x4 - x2)x(x3 - x2)
                        REAL3 cross = math.cross(id1_minus_id0, id2_minus_id0);

                        //Multiplying by 1/6 in the denominator is the same as multiplying by 6 in the numerator
                        //Im not sure why hes doing it... maybe because alpha should not be affected by it?  
                        REAL3 gradC = cross * (1f / 6f);

                        gradients[j] = gradC;

                        //w1 * |grad_C1|^2
                        wTimesGrad += invMass[idThis] * math.lengthsq(gradC);
                    }

                    //All vertices are fixed so dont simulate
                    if (wTimesGrad <= Util.EPSILON)
                    {
                        continue;
                    }

                    REAL vol = GetTetVolume(i);
                    REAL restVol = restVolumes[i];

                    REAL C = vol - restVol;

                    //The guy in the video is dividing by 6 in the code but multiplying in the video
                    //C *= 6f;

                    REAL lambda = -C / (wTimesGrad + alpha);

                    //Move each vertex
                    for (int j = 0; j < 4; j++)
                    {
                        int id = tets[i][j];

                        //Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
                        pos[id] += lambda * invMass[id] * gradients[j];
                    }
                }

            }

            private REAL GetTetVolume(int index)
            {
                //The 4 vertices belonging to this tetra 
                int id0 = tets[index][0];
                int id1 = tets[index][1];
                int id2 = tets[index][2];
                int id3 = tets[index][3];

                REAL3 a = pos[id0];
                REAL3 b = pos[id1];
                REAL3 c = pos[id2];
                REAL3 d = pos[id3];

                REAL3 d0 = b - a;
                REAL3 d1 = c - a;
                REAL3 d2 = d - a;

                REAL volume = math.dot(math.cross(d1, d2), d0) * (1f / 6f);
                return volume;
            }
        }

        [BurstCompile]
        private struct UpdateVisMeshJob : IJobParallelFor
        {
            public NativeArray<Vector3> visPos;
            [ReadOnly]
            public NativeArray<REAL3> pos;
            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<REAL4> skinningInfo;

            public void Execute(int index)
            {
                int tetNr = (int)skinningInfo[index][0];
                if (tetNr < 0)
                    return;

                REAL b0 = skinningInfo[index][1];
                REAL b1 = skinningInfo[index][2];
                REAL b2 = skinningInfo[index][3];
                REAL b3 = 1f - b0 - b1 - b2;

                int id0 = tets[tetNr].x;
                int id1 = tets[tetNr].y;
                int id2 = tets[tetNr].z;
                int id3 = tets[tetNr].w;

                visPos[index] = Vector3.zero;
                visPos[index] = (float3)visPos[index] + (float3)(pos[id0] * b0);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id1] * b1);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id2] * b2);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id3] * b3);
            }
        }
        #endregion
    }

}

