using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using System.Collections.Generic;
using System;

namespace XPBD
{
    public unsafe class SoftBody : Body
    {
        public VisMesh visMesh;
        public PhysicMesh physicMesh;
        private Material material;
        private Shader shader1, shader2;

        public float mu = 10f, lambda = 1000f;
        public bool showTet = false;
        public bool showVis = true;

        // Simulation Data NativeArray
        public NativeArray<float3> Pos;
        public NativeArray<float3> prevPos;
        public NativeArray<float3> vel;
        private NativeArray<int4> tets;
        public NativeArray<float> invMass;
        private NativeArray<float> restVolumes;
        private NativeArray<float3x3> invDm;

        // Skinning Info
        private NativeArray<float3> visPos;
        private NativeArray<float4> skinningInfo;

        public int VerticesNum { get; private set; }
        public int TetsNum { get; private set; }

        private float3 startPos;
        [SerializeField]
        private Vector3 v0 = Vector3.zero;

        private JobHandle jobHandle;

        private struct Collision
        {
            public int index;
            public float frictionCoef;
            public float3 q;
            public float3 N;
            public float3 fn;
            public float vn_;
            public Primitive primitive;
        }

        private List<Collision> collisions;

        public override void CollectCollision(float dt, Primitive primitive)
        {
            collisions.Clear();
            float3 surfaceN = new(0f, 1f, 0f);
            for (int i = 0; i < VerticesNum; ++i)
            {
                float3 predPos = Pos[i] + vel[i] * dt;
                if (predPos.y <= 0.0f)
                {
                    Collision newCollision = new();
                    newCollision.index = i;
                    newCollision.N = surfaceN;
                    newCollision.vn_ = math.dot(surfaceN, vel[i]);
                    newCollision.frictionCoef = 0.4f;
                    newCollision.fn = float3.zero;
                    if (Pos[i].y > 0.0f)
                    {
                        float3 ray = predPos - Pos[i];
                        float coef = -Pos[i].y / ray.y;
                        float3 q = Pos[i] + coef * ray;
                        newCollision.q = q;
                    }
                    else if (Pos[i].y <= 0.0f)
                    {
                        float3 q = predPos;
                        q.y = 0.0f;
                        newCollision.q = q;
                    }
                    collisions.Add(newCollision);
                }
            }
        }

        public override void PreSolve(float dt, Vector3 gravity)
        {
            float3 g = float3.zero;
            if (UseGravity)
                g = gravity;

            PreSolveJob preSolveJob = new PreSolveJob
            {
                dt = dt,
                gravity = g,
                pos = Pos,
                prevPos = prevPos,
                vel = vel,
                invMass = invMass
            };
            jobHandle = preSolveJob.Schedule(VerticesNum, 1, jobHandle);
            jobHandle.Complete();
        }

        public override void Solve(float dt)
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
            jobHandle = solveElementJob.Schedule(TetsNum, jobHandle);
            jobHandle.Complete();
            SolveCollision(dt);
        }

        public override void PostSolve(float dt)
        {
            for (int i = 0; i < VerticesNum; ++i)
            {
                // Ignore zero mass
                if (invMass[i] == 0)
                    continue;

                vel[i] = (Pos[i] - prevPos[i]) / dt;
            }
        }

        public override void VelocitySolve(float dt)
        {
            float restitutionCoef;
            for (int i = 0; i < collisions.Count; i++)
            {
                Collision c = collisions[i];
                float3 v = vel[c.index];
                float vn = math.dot(c.N, v);
                float3 vt = v - vn * c.N;

                // tangent
                float3 dvt = -math.normalizesafe(vt, float3.zero) * math.min(dt * c.frictionCoef * math.length(c.fn), math.length(vt));
                vel[c.index] += dvt;

                // normal
                restitutionCoef = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
                float3 dvn = c.N * (-vn + math.max(0.0f, -restitutionCoef * c.vn_));
                vel[c.index] += dvn;
            }
        }
        public override void EndFrame()
        {
            physicMesh.Show(showTet);
            visMesh.Show(showVis);

            physicMesh.UpdateMesh(Pos);

            UpdateVisMeshJob updateVisMeshJob = new UpdateVisMeshJob
            {
                visPos = visPos,
                pos = Pos,
                tets = tets,
                skinningInfo = skinningInfo
            };
            jobHandle = updateVisMeshJob.Schedule(visPos.Length, 1, jobHandle);
            jobHandle.Complete();

            visMesh.UpdateMesh(visPos);

        }

        #region IGrabbable
        public override void StartGrab(Vector3 grabPos)
        {

        }

        public override void MoveGrabbed(Vector3 grabPos)
        { }

        public override void EndGrab(Vector3 grabPos, Vector3 vel)
        { }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            hit = null;
        }

        public override Vector3 GetGrabbedPos()
        {
            return Vector3.zero;
        }
        #endregion

        private void Awake()
        {
            visMesh.Initialize();
            physicMesh.Initialize();
            material = visMesh.meshRenderer.material;
            shader1 = material.shader;
            shader2 = Shader.Find("Custom/NewUnlitShader");
            Initialize();
            //ID = BackEnd.AddXPBDSoftBody(visMesh.state, physicMesh.state, transform.position, transform.rotation, mass, mu, lambda);
            //Debug.Log(ID);
            Debug.Log("SoftBody Awake");
        }
        private void Start()
        {
            Simulation.get.AddBody(this);
            Debug.Log("SoftBody Start");
        }

        private void Update()
        {
            //if (Input.GetKeyDown(KeyCode.Space))
            //{
            //    if (material.shader == shader1)
            //    {
            //        material.shader = shader2;
            //        Vector4 vector = UnityEngine.Random.insideUnitSphere;
            //        //vector.w = 1;
            //        material.SetVector("_MarcoNormal", vector);
            //    }
            //    else
            //    {
            //        material.shader = shader1;
            //    }
            //}
        }


        private void OnDestroy()
        {
            if (Pos.IsCreated) Pos.Dispose();
            if (prevPos.IsCreated) prevPos.Dispose();
            if (vel.IsCreated) vel.Dispose();
            if (invMass.IsCreated) invMass.Dispose();
            if (tets.IsCreated) tets.Dispose();
            if (restVolumes.IsCreated) restVolumes.Dispose();
            if (invDm.IsCreated) invDm.Dispose();
            if (visPos.IsCreated) visPos.Dispose();
            if (skinningInfo.IsCreated) skinningInfo.Dispose();
        }

        private void Initialize()
        {
            bodyType = BodyType.Soft;

            VerticesNum = physicMesh.mesh.vertices.Length;
            TetsNum = physicMesh.tets.Length / 4;

            Pos = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            prevPos = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            vel = new NativeArray<float3>(VerticesNum, Allocator.Persistent);
            invMass = new NativeArray<float>(VerticesNum, Allocator.Persistent);

            tets = new NativeArray<int4>(TetsNum, Allocator.Persistent);
            restVolumes = new NativeArray<float>(TetsNum, Allocator.Persistent);
            invDm = new NativeArray<float3x3>(TetsNum, Allocator.Persistent);

            // Initialize NativeArrays
            for (int i = 0; i < VerticesNum; ++i)
            {
                Pos[i] = prevPos[i] = physicMesh.mesh.vertices[i];
                vel[i] = v0;
                invMass[i] = 0f;
            }
            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = physicMesh.tets[4 * i + 0];
                int id1 = physicMesh.tets[4 * i + 1];
                int id2 = physicMesh.tets[4 * i + 2];
                int id3 = physicMesh.tets[4 * i + 3];

                tets[i] = new int4(id0, id1, id2, id3);
            }
            for (int i = 0; i < TetsNum; ++i)
            {
                restVolumes[i] = 0f;
                invDm[i] = float3x3.identity;
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
            visPos = new NativeArray<float3>(visMesh.mesh.vertexCount, Allocator.Persistent);
            ComputeSkinningInfo();

            // Move to starting position
            startPos = transform.position;
            transform.position = Vector3.zero;
            Translate(startPos);

            collisions = new List<Collision>();
        }

        private void ComputeSkinningInfo()
        {
            int numVisVerts = visMesh.mesh.vertexCount;

            Hash hash = new Hash(0.15f, numVisVerts);
            hash.Create(visMesh.mesh.vertices);

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
            float averageEdge = 0f;
            for(int i = 0; i < TetsNum; ++i)
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

                    float3 visVert = visMesh.mesh.vertices[id];
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
            averageEdge /= TetsNum;
            // Debug.Log("averageEdge " + averageEdge);

            // Print skinningInfo
            //string str = string.Empty;
            //for (int i = 0; i < skinningInfo.Length; ++i)
            //{
            //    str += skinningInfo[i] + "\n";
            //}
            //Debug.Log("skinningInfo:\n" + str);
        }

        private void SolveCollision(float dt)
        {
            for (int i = 0; i < collisions.Count; i++)
            {
                Collision c = collisions[i];
                float C = math.dot(Pos[c.index] - c.q, c.N);
                if (C >= 0.0f)
                    continue;

                float alpha = 0.0f;
                float w1 = invMass[c.index];
                float w2 = 0.0f;
                float dlambda = -C / (w1 + w2 + alpha);
                float3 p = dlambda * c.N;
                c.fn = p / (dt * dt);
                collisions[i] = c;
                Pos[c.index] += p * w1;


                float3 dp = Pos[c.index] - prevPos[c.index];
                float3 dp_t = dp - math.dot(dp, c.N) * c.N;

                float C2 = math.length(dp_t);
                if (C2 <= 1e-6f)
                    continue;

                float dlambda_t = -C2 / (w1 + w2 + alpha);
                dlambda_t = math.min(dlambda_t, dlambda * c.frictionCoef);
                float3 p2 = dlambda_t * math.normalizesafe(dp_t, float3.zero);
                Pos[c.index] += p2 * w1;
            }
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
        private struct SolveElementJob : IJobFor
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
            public void Execute(int index)
            {
                //for (int index = 0; index < tets.Length; ++index)
                //{
                //    SolveDeviatoric(index);
                //    SolveVolumetric(index);
                //}
                SolveDeviatoric(index);
                SolveVolumetric(index);
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

