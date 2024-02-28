using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using System.Collections.Generic;
using System;
using UnityEngine.Assertions;

namespace XPBD
{
    [RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public class SoftBody : Body
    {
        //public VisMesh visMesh;
        //public PhysicMesh physicMesh;
        public TetrahedronMesh tetrahedronMesh;

        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        private Mesh visualMesh;
        private Mesh tetMesh;
        private Material visualMaterial;
        public Material wireframeMaterial;

        public float mu = 10f, lambda = 1000f;
        public bool showTet = false;
        //public bool showVis = true;

        // Simulation Data NativeArray
        public NativeArray<float3> Pos;
        public NativeArray<float3> prevPos;
        public NativeArray<float3> Vel;
        private NativeArray<int4> tets;
        public NativeArray<float> invMass;
        private NativeArray<float> restVolumes;
        private NativeArray<float3x3> invDm;
        private NativeArray<int2> edges;
        private NativeArray<float> restEdgeLengths;

        private NativeArray<int3> volIdOrder;
        public float3 fext { get; set; }

        // Skinning Info
        private NativeArray<float3> visPos;
        private NativeArray<float4> skinningInfo;

        public int VerticesNum { get; private set; }
        public int TetsNum { get; private set; }

        private int EdgesNum;

        private float3 startPos;
        [SerializeField]
        private Vector3 v0 = Vector3.zero;

        private JobHandle jobHandle;

        private List<MyCollision> collisions;

        // Grabber info
        public int grabId { get; private set; }
        private float grabInvMass;
        

        #region Body
        public override void ClearCollision()
        {
            
        }

        public override void PreSolve(float dt, Vector3 gravity)
        {
            float3 g = float3.zero;
            if (UseGravity)
                g = gravity;

            PreSolveJob preSolveJob = new PreSolveJob
            {
                dt = dt,
                gravity = g + fext,
                pos = Pos,
                prevPos = prevPos,
                vel = Vel,
                invMass = invMass
            };
            jobHandle = preSolveJob.Schedule(VerticesNum, 1, jobHandle);
            jobHandle.Complete();
        }

        public override void Solve(float dt)
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

        public override void PostSolve(float dt)
        {
            for (int i = 0; i < VerticesNum; ++i)
            {
                // Ignore zero mass
                if (invMass[i] == 0)
                    continue;

                Vel[i] = (Pos[i] - prevPos[i]) / dt;
            }
        }

        public override void VelocitySolve(float dt)
        {
        }
        public override void EndFrame()
        {
            //physicMesh.Show(showTet);
            //visMesh.Show(showVis);
            //physicMesh.UpdateMesh(Pos);

            UpdateVisMeshJob updateVisMeshJob = new UpdateVisMeshJob
            {
                visPos = visPos,
                pos = Pos,
                tets = tets,
                skinningInfo = skinningInfo
            };
            jobHandle = updateVisMeshJob.Schedule(visPos.Length, 1, jobHandle);
            jobHandle.Complete();

            //visMesh.UpdateMesh(visPos);

            visualMesh.SetVertices(visPos);
            visualMesh.RecalculateBounds();
            visualMesh.RecalculateNormals();
            visualMesh.RecalculateTangents();

            tetMesh.SetVertices(Pos);
            tetMesh.RecalculateBounds();
            tetMesh.RecalculateNormals();
            tetMesh.RecalculateTangents();

            if (showTet)
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
        public override void StartGrab(Vector3 grabPos)
        {
            //Find the closest vertex to the pos on a triangle in the mesh
            float minD2 = float.MaxValue;
            float3 gPos = grabPos;
            grabId = -1;

            for (int i = 0; i < VerticesNum; i++)
            {

                float d2 = math.lengthsq(gPos - Pos[i]);

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
                invMass[grabId] = 0f;

                //Set the position of the vertex to the position where the ray hit the triangle
                Pos[grabId] = grabPos;
            }
        }

        public override void MoveGrabbed(Vector3 grabPos)
        {
            if (grabId >= 0)
            {
                Pos[grabId] = grabPos;
            }
        }

        public override void EndGrab(Vector3 grabPos, Vector3 vel)
        {
            if (grabId >= 0)
            {
                //Set the mass to whatever mass it was before we grabbed it
                invMass[grabId] = grabInvMass;

                this.Vel[grabId] = vel;
            }

            grabId = -1;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            float3[] vertices = Pos.ToArray();
            //int[] triangles = physicMesh.mesh.triangles;
            int[] triangles = tetrahedronMesh.faces;

            Intersection.IsRayHittingMesh(ray, vertices, triangles, out hit);
        }

        public override Vector3 GetGrabbedPos()
        {
            return Pos[grabId];
        }
        #endregion

        #region Monobehaviour
        private void Awake()
        {
            //visMesh.Initialize();
            //physicMesh.Initialize();

            Assert.IsNotNull(tetrahedronMesh);

            InitializeMesh();
            //InitializePhysics();

            InitializePhysics2();
            Debug.Log("SoftBody Awake");
        }
        private void Start()
        {
            Simulation.get.AddBody(this);
            Debug.Log("SoftBody Start");
        }

        private void OnDestroy()
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
        }
        #endregion

        private void InitializeMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();

            visualMesh = meshFilter.mesh;

            // Create tetmesh from tetrahedron data
            tetMesh = new Mesh();
            tetMesh.name = gameObject.name + "_tetmesh";
            tetMesh.SetVertices(tetrahedronMesh.vertices);
            tetMesh.SetIndices(tetrahedronMesh.faces, MeshTopology.Triangles, 0);
            tetMesh.MarkDynamic();
            tetMesh.MarkModified();
            tetMesh.RecalculateBounds();
            tetMesh.RecalculateNormals();

            // Set materials
            visualMaterial = meshRenderer.material;

        }
        private void InitializePhysics2()
        {
            bodyType = BodyType.Soft;

            VerticesNum = tetrahedronMesh.vertices.Length;
            TetsNum = tetrahedronMesh.tets.Length / 4;
            EdgesNum = tetrahedronMesh.edges.Length / 2;

            Pos = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            prevPos = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            Vel = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            invMass = new NativeArray<float>(VerticesNum, Allocator.Persistent);

            tets = new NativeArray<int4>(TetsNum, Allocator.Persistent);
            restVolumes = new NativeArray<float>(TetsNum, Allocator.Persistent);
            invDm = new NativeArray<float3x3>(TetsNum, Allocator.Persistent);

            edges = new NativeArray<int2>(EdgesNum, Allocator.Persistent);
            restEdgeLengths = new NativeArray<float>(EdgesNum, Allocator.Persistent);

            volIdOrder = new NativeArray<int3>(4, Allocator.Persistent);
            volIdOrder[0] = new(1, 3, 2);
            volIdOrder[1] = new(0, 2, 3);
            volIdOrder[2] = new(0, 3, 1);
            volIdOrder[3] = new(0, 1, 2);

            // Initialize NativeArrays
            for (int i = 0; i < VerticesNum; ++i)
            {
                Pos[i] = prevPos[i] = tetrahedronMesh.vertices[i];
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
                invDm[i] = float3x3.identity;
            }
            for(int i = 0; i < EdgesNum; ++i)
            {
                int id0 = tetrahedronMesh.edges[2 * i + 0];
                int id1 = tetrahedronMesh.edges[2 * i + 1];

                edges[i] = new int2(id0, id1);
                restEdgeLengths[i] = math.length(Pos[id0] - Pos[id1]);
            }

            // Rest volume
            float totalVolume = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;

                float3x3 RestPose = new float3x3(Pos[id1] - Pos[id0], Pos[id2] - Pos[id0], Pos[id3] - Pos[id0]);
                invDm[i] = math.inverse(RestPose);
                float V = math.determinant(RestPose);

                float partialV = V / 4.0f;
                invMass[id0] += partialV;
                invMass[id1] += partialV;
                invMass[id2] += partialV;
                invMass[id3] += partialV;

                restVolumes[i] = V;
                totalVolume += V;
            }

            // Inverse mass (1/w)
            float density = mass / totalVolume;
            for (int i = 0; i < VerticesNum; ++i)
            {
                invMass[i] *= density;
                if (invMass[i] != 0.0f)
                    invMass[i] = 1.0f / invMass[i];
            }

            // Visual embedded mesh
            visPos = new NativeArray<float3>(visualMesh.vertexCount, Allocator.Persistent);
            ComputeSkinningInfo2();

            // Move to starting position
            startPos = transform.position;
            transform.position = Vector3.zero;
            Translate(startPos);
        }

        private void ComputeSkinningInfo2()
        {
            int numVisVerts = visualMesh.vertexCount;

            Hash hash = new Hash(0.15f, numVisVerts);
            hash.Create(visualMesh.vertices);

            skinningInfo = new NativeArray<float4>(numVisVerts, Allocator.Persistent);
            for (int i = 0; i < skinningInfo.Length; ++i)
                skinningInfo[i] = new float4(-1, -1, -1, -1);

            float[] minDist = new float[numVisVerts];
            Array.Fill(minDist, float.MaxValue);
            float border = 0.05f;

            // Each tet searches for containing vertices
            float3 tetCenter = float3.zero;
            float3x3 mat = float3x3.zero;
            float4 bary = float4.zero;
            //float averageEdge = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                // Compute bounding sphere for tet
                tetCenter = float3.zero;
                for (int j = 0; j < 4; ++j)
                    tetCenter += 0.25f * (Pos[tets[i][j]]);

                float rMax = 0f;

                for (int j = 0; j < 4; ++j)
                {
                    float r2 = math.length(tetCenter - Pos[tets[i][j]]);
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

                //float edgeLength = math.length(Pos[id0] - Pos[id3]);
                //edgeLength += math.length(Pos[id1] - Pos[id3]);
                //edgeLength += math.length(Pos[id2] - Pos[id3]);
                //edgeLength += math.length(Pos[id0] - Pos[id2]);
                //edgeLength += math.length(Pos[id1] - Pos[id2]);
                //edgeLength += math.length(Pos[id0] - Pos[id1]);
                //averageEdge += edgeLength / 6;

                mat.c0 = Pos[id0] - Pos[id3];
                mat.c1 = Pos[id1] - Pos[id3];
                mat.c2 = Pos[id2] - Pos[id3];

                mat = math.inverse(mat);

                for (int j = 0; j < hash.querySize; ++j)
                {
                    int id = hash.queryIds[j];

                    if (minDist[id] <= 0f)
                        continue;

                    float3 visVert = visualMesh.vertices[id];
                    if (math.lengthsq(visVert - tetCenter) > rMax * rMax)
                        continue;

                    bary.xyz = visVert - Pos[id3];
                    bary.xyz = math.mul(mat, bary.xyz);
                    bary.w = 1f - bary.x - bary.y - bary.z;

                    float dist = 0f;

                    for (int k = 0; k < 4; ++k)
                        dist = math.max(dist, -bary[k]);

                    if (dist < minDist[id])
                    {
                        minDist[id] = dist;
                        skinningInfo[id] = new float4(i, bary[0], bary[1], bary[2]);
                    }
                }
            }
            // averageEdge /= TetsNum;
            // Debug.Log("averageEdge " + averageEdge);

            // Print skinningInfo
            //string str = string.Empty;
            //for (int i = 0; i < skinningInfo.Length; ++i)
            //{
            //    str += skinningInfo[i] + "\n";
            //}
            //Debug.Log("skinningInfo:\n" + str);
        }

        private void Translate(float3 moveDist)
        {
            for (int i = 0; i < VerticesNum; i++)
            {
                Pos[i] += moveDist;
                prevPos[i] += moveDist;
            }
        }


        #region IJob
        [BurstCompile]
        private struct PreSolveJob : IJobParallelFor
        {
            public float dt;
            public float3 gravity;

            public NativeArray<float3> pos;
            public NativeArray<float3> prevPos;
            public NativeArray<float3> vel;

            [ReadOnly]
            public NativeArray<float> invMass;

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
            public float dt;
            public float mu;
            public float lambda;

            public NativeArray<float3> pos;

            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<float> restVolumes;
            [ReadOnly]
            public NativeArray<float> invMass;
            [ReadOnly]
            public NativeArray<float3x3> invDm;

            private float3x4 gradients;
            private float3x3 F, dF;
            public void Execute()
            {
                for (int index = 0; index < tets.Length; ++index)
                {
                    SolveDeviatoric(index);
                    SolveVolumetric(index);
                }
                //SolveDeviatoric(index);
                //SolveVolumetric(index);
            }

            private float3x3 GetDeformationGradient(int tetIndex)
            {
                int id0 = tets[tetIndex].x;
                int id1 = tets[tetIndex].y;
                int id2 = tets[tetIndex].z;
                int id3 = tets[tetIndex].w;

                float3x3 Ds = float3x3.zero;
                Ds.c0 = pos[id1] - pos[id0];
                Ds.c1 = pos[id2] - pos[id0];
                Ds.c2 = pos[id3] - pos[id0];

                return math.mul(Ds, invDm[tetIndex]);
            }
            private void SolveDeviatoric(int i)
            {
                float compliance = 1f / mu / restVolumes[i];

                F = GetDeformationGradient(i);
                float r_s = math.sqrt(math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2));
                float C = r_s;

                if (C == 0f)
                    return;

                float r_s_inv = 1f / r_s;

                // gradients set zero
                gradients = float3x4.zero;

                gradients.c1 += r_s_inv * invDm[i].c0.x * F.c0;
                gradients.c1 += r_s_inv * invDm[i].c1.x * F.c1;
                gradients.c1 += r_s_inv * invDm[i].c2.x * F.c2;

                gradients.c2 += r_s_inv * invDm[i].c0.y * F.c0;
                gradients.c2 += r_s_inv * invDm[i].c1.y * F.c1;
                gradients.c2 += r_s_inv * invDm[i].c2.y * F.c2;

                gradients.c3 += r_s_inv * invDm[i].c0.z * F.c0;
                gradients.c3 += r_s_inv * invDm[i].c1.z * F.c1;
                gradients.c3 += r_s_inv * invDm[i].c2.z * F.c2;

                gradients.c0 = -gradients.c1 - gradients.c2 - gradients.c3;

                ApplyToElement(i, C, compliance, dt);
            }

            private void SolveVolumetric(int i)
            {
                float gamma = mu / lambda;
                float compliance = 1f / lambda / restVolumes[i];

                F = GetDeformationGradient(i);
                float C = math.determinant(F) - 1f - gamma;

                if (C == 0f)
                    return;

                dF = float3x3.zero;
                dF.c0 = math.cross(F.c1, F.c2);
                dF.c1 = math.cross(F.c2, F.c0);
                dF.c2 = math.cross(F.c0, F.c1);

                // gradients set zero
                gradients = float3x4.zero;
                gradients.c1 += invDm[i].c0.x * dF.c0;
                gradients.c1 += invDm[i].c1.x * dF.c1;
                gradients.c1 += invDm[i].c2.x * dF.c2;

                gradients.c2 += invDm[i].c0.y * dF.c0;
                gradients.c2 += invDm[i].c1.y * dF.c1;
                gradients.c2 += invDm[i].c2.y * dF.c2;

                gradients.c3 += invDm[i].c0.z * dF.c0;
                gradients.c3 += invDm[i].c1.z * dF.c1;
                gradients.c3 += invDm[i].c2.z * dF.c2;

                gradients.c0 = -gradients.c1 - gradients.c2 - gradients.c3;

                ApplyToElement(i, C, compliance, dt);

            }
            private void ApplyToElement(int i, float C, float compliance, float dt)
            {
                float weight = 0f;
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;
                weight += math.lengthsq(gradients.c0) * invMass[id0];
                weight += math.lengthsq(gradients.c1) * invMass[id1];
                weight += math.lengthsq(gradients.c2) * invMass[id2];
                weight += math.lengthsq(gradients.c3) * invMass[id3];

                if (weight == 0f)
                    return;

                float h2 = dt * dt;
                float alpha = compliance / h2;
                float dlambda = -C / (weight + alpha);

                id0 = tets[i].x;
                id1 = tets[i].y;
                id2 = tets[i].z;
                id3 = tets[i].w;
                pos[id0] += dlambda * invMass[id0] * gradients.c0;
                pos[id1] += dlambda * invMass[id1] * gradients.c1;
                pos[id2] += dlambda * invMass[id2] * gradients.c2;
                pos[id3] += dlambda * invMass[id3] * gradients.c3;
            }
        }

        [BurstCompile]
        private struct SolveElementJob2 : IJob
        {
            public float dt;
            public float mu;
            public float lambda;

            public NativeArray<float3> pos;

            [ReadOnly]
            public NativeArray<int2> edges;
            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<float> restEdgeLengths;
            [ReadOnly]
            public NativeArray<float> restVolumes;
            [ReadOnly]
            public NativeArray<float> invMass;

            private float3x4 gradients;

            public NativeArray<int3> volIdOrder;

            public void Execute()
            {
                SolveEdges();
                SolveVolumes();
            }

            private void SolveEdges()
            {
                float compliance = 1f / mu;
                float alpha = compliance / (dt * dt);

                //For each edge
                for (int i = 0; i < edges.Length; ++i)
                {
                    //2 vertices per edge in the data structure, so multiply by 2 to get the correct vertex index
                    int id0 = edges[i].x;
                    int id1 = edges[i].y;

                    float w0 = invMass[id0];
                    float w1 = invMass[id1];

                    float wTot = w0 + w1;

                    //This edge is fixed so dont simulate
                    if (wTot < Util.EPSILON)
                        continue;

                    //x0-x1
                    //The result is stored in grads array
                    float3 id0_minus_id1 = pos[id0] - pos[id1];

                    //sqrMargnitude(x0-x1)
                    float l = math.length(id0_minus_id1);

                    //If they are at the same pos we get a divisio by 0 later so ignore
                    if (l < Util.EPSILON)
                    {
                        continue;
                    }

                    //(x0-x1) * (1/|x0-x1|) = gradC
                    float3 gradC = id0_minus_id1 / l;

                    float l_rest = restEdgeLengths[i];

                    float C = l - l_rest;

                    //lambda because |grad_Cn|^2 = 1 because if we move a particle 1 unit, the distance between the particles also grows with 1 unit, and w = w0 + w1
                    float lambda = -C / (wTot + alpha);

                    //Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
                    pos[id0] += lambda * w0 * gradC;
                    pos[id1] += -lambda * w1 * gradC;
                }
            }

            private void SolveVolumes()
            {
                float compliance = 1f / lambda;
                float alpha = compliance / (dt * dt);

                for (int i = 0; i < tets.Length; ++i)
                {
                    float wTimesGrad = 0f;

                    //Foreach vertex in the tetra
                    for (int j = 0; j < 4; j++)
                    {
                        int idThis = tets[i][j];

                        //The 3 opposite vertices ids
                        int id0 = tets[i][volIdOrder[j][0]];
                        int id1 = tets[i][volIdOrder[j][1]];
                        int id2 = tets[i][volIdOrder[j][2]];

                        //(x4 - x2)
                        float3 id1_minus_id0 = pos[id1] - pos[id0];
                        //(x3 - x2)
                        float3 id2_minus_id0 = pos[id2] - pos[id0];

                        //(x4 - x2)x(x3 - x2)
                        float3 cross = math.cross(id1_minus_id0, id2_minus_id0);

                        //Multiplying by 1/6 in the denominator is the same as multiplying by 6 in the numerator
                        //Im not sure why hes doing it... maybe because alpha should not be affected by it?  
                        float3 gradC = cross * (1f / 6f);

                        gradients[j] = gradC;

                        //w1 * |grad_C1|^2
                        wTimesGrad += invMass[idThis] * math.lengthsq(gradC);
                    }

                    //All vertices are fixed so dont simulate
                    if (wTimesGrad <= Util.EPSILON)
                    {
                        continue;
                    }

                    float vol = GetTetVolume(i);
                    float restVol = restVolumes[i];

                    float C = vol - restVol;

                    //The guy in the video is dividing by 6 in the code but multiplying in the video
                    //C *= 6f;

                    float lambda = -C / (wTimesGrad + alpha);

                    //Move each vertex
                    for (int j = 0; j < 4; j++)
                    {
                        int id = tets[i][j];

                        //Move the vertices x = x + deltaX where deltaX = lambda * w * gradC
                        pos[id] += lambda * invMass[id] * gradients[j];
                    }
                }

            }

            private float GetTetVolume(int index)
            {
                //The 4 vertices belonging to this tetra 
                int id0 = tets[index][0];
                int id1 = tets[index][1];
                int id2 = tets[index][2];
                int id3 = tets[index][3];

                float3 a = pos[id0];
                float3 b = pos[id1];
                float3 c = pos[id2];
                float3 d = pos[id3];

                float3 d0 = b - a;
                float3 d1 = c - a;
                float3 d2 = d - a;

                float volume = math.dot(math.cross(d1, d2), d0) * (1f / 6f);
                return volume;
            }
        }

        [BurstCompile]
        private struct UpdateVisMeshJob : IJobParallelFor
        {
            public NativeArray<float3> visPos;
            [ReadOnly]
            public NativeArray<float3> pos;
            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<float4> skinningInfo;

            public void Execute(int index)
            {
                int tetNr = (int)skinningInfo[index][0];
                if (tetNr < 0)
                    return;

                float b0 = skinningInfo[index][1];
                float b1 = skinningInfo[index][2];
                float b2 = skinningInfo[index][3];
                float b3 = 1f - b0 - b1 - b2;

                int id0 = tets[tetNr].x;
                int id1 = tets[tetNr].y;
                int id2 = tets[tetNr].z;
                int id3 = tets[tetNr].w;

                visPos[index] = float3.zero;
                visPos[index] += pos[id0] * b0;
                visPos[index] += pos[id1] * b1;
                visPos[index] += pos[id2] * b2;
                visPos[index] += pos[id3] * b3;
            }
        }
        #endregion
    }

}

