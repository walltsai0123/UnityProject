using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;
using Unity.Jobs;
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
    public class SoftBodySystem : IDisposable
    {
        List<SoftBody> softBodies = new();

        List<int> verticesOffset;
        List<int> tetsOffset;

        public int VerticesNum { get; private set; } = 0;
        public int TetsNum { get; private set; } = 0;
        public int FacesNum { get; private set; } = 0;

        public NativeArray<SoftBodyParticle> particles;
        NativeArray<ElementConstraint> elementConstraints;
        public NativeArray<int3> surfaceFaces;
        NativeArray<int> clusters;
        NativeArray<int> clusterSizes;
        NativeArray<int> clusterStarts;
        List<int> passes = new();

        JobHandle jobHandle;
        public void Dispose()
        {
            particles.Dispose();
            elementConstraints.Dispose();
            surfaceFaces.Dispose();
            
            clusters.Dispose();
            clusterSizes.Dispose();
            clusterStarts.Dispose();
        }

        public void AddSoftBody(SoftBody softBody)
        {
            softBodies ??= new List<SoftBody>();
            softBodies.Add(softBody);
            
            verticesOffset ??= new List<int>();
            verticesOffset.Add(VerticesNum);

            tetsOffset ??= new List<int>();
            tetsOffset.Add(TetsNum);

            VerticesNum += softBody.VerticesNum;
            TetsNum += softBody.TetsNum;
            FacesNum += softBody.FacesNum;
        }

        public SoftBody GetParticleBody(int index)
        {
            if(index < 0 || index >= VerticesNum) return null;

            for(int i = 0; i < verticesOffset.Count - 1; i++)
            {
                int bodyStart = verticesOffset[i];
                int bodyEnd = verticesOffset[i + 1];

                if(index >= bodyStart && index < bodyEnd)
                    return softBodies[i];
            }

            return softBodies[softBodies.Count - 1];
        }

        public NativeSlice<SoftBodyParticle> GetBodyParticles(SoftBody softBody)
        {
            int index = softBodies.IndexOf(softBody);
            if (index < 0)
                return new();

            return new(particles, verticesOffset[index], softBody.VerticesNum);
        }

        public void Init()
        {

            if (softBodies?.Count == 0)
                return;
            
            particles = new NativeArray<SoftBodyParticle>(VerticesNum, Allocator.Persistent);
            elementConstraints = new NativeArray<ElementConstraint>(TetsNum, Allocator.Persistent);
            surfaceFaces = new NativeArray<int3>(FacesNum, Allocator.Persistent);

            int faceStartIndex = 0;
            for (int i = 0; i < softBodies.Count; i++)
            {
                SoftBody b = softBodies[i];
                int startIndex = verticesOffset[i];
                particles.GetSubArray(startIndex, b.VerticesNum).CopyFrom(b.particles.ToArray());

                startIndex = tetsOffset[i];

                elementConstraints.GetSubArray(startIndex, b.TetsNum)
                    .CopyFrom(b.elementConstraints.Select(x=>
                    {
                        x.tet += verticesOffset[i];
                        return x;
                    }).ToArray());

                surfaceFaces.GetSubArray(faceStartIndex, b.FacesNum)
                    .CopyFrom(b.surfaceFaces.Select(x=>
                    {
                        x += verticesOffset[i];
                        return x;
                    }).ToArray());
                faceStartIndex += b.FacesNum;
            }
            ElementConstraintClustering();
        }

        public void SyncBodies()
        {
            // Sync body data
            for (int i = 0; i < softBodies.Count; i++)
            {
                int startIndex = verticesOffset[i];
                SoftBody b = softBodies[i];
                particles.GetSubArray(startIndex, b.VerticesNum).CopyFrom(b.particles.ToArray());
            }
        }
        public void PreSolve(REAL dt, REAL3 gravity)
        {
            //Debug.Log($"{particles[0].f_ext}");
            PreSolveJob preSolveJob = new()
            {
                dt = dt,
                gravity = gravity,
                particles = particles,
            };
            jobHandle = preSolveJob.Schedule(VerticesNum, 1, jobHandle);
            jobHandle.Complete();
        }

        public void Solve(REAL dt)
        {
            if(Simulation.get.parallelBlockXPBD)
            {
                int first = 0;
                for (int i = 0; i < passes.Count; i++)
                {
                    SolveElementJob solveElementJob = new()
                    {
                        dt = dt,
                        particles = particles,
                        elementConstraints = elementConstraints,
                        cluster = clusters,
                        clusterSizes = clusterSizes,
                        clusterStart = clusterStarts,
                        firstCluster = first
                    };
                    jobHandle = solveElementJob.Schedule(passes[i], 1, jobHandle);
                    jobHandle.Complete();
                    first += passes[i];
                }
            }
            else
            {
                SolveElementJob2 solveElementJob = new()
                {
                    dt = dt,
                    particles = particles,
                    elementConstraints = elementConstraints,
                };
                jobHandle = solveElementJob.Schedule(jobHandle);
                jobHandle.Complete();
            }
        }

        public void PostSolve(REAL dt)
        {
            PostSolveJob postSolveJob = new()
            {
                dt = dt,
                particles = particles,
            };
            jobHandle = postSolveJob.Schedule(VerticesNum, 1, jobHandle);
            jobHandle.Complete();
        }

        public void ClearForce()
        {
            for (int i = 0; i < VerticesNum; i++)
            {
                var p = particles[i];
                p.f_ext = 0;
                particles[i] = p;
            }
        }

        public void EndFrame()
        {
            for(int i = 0; i < softBodies.Count; i++)
            {
                //int offset = verticesOffset[i];
                var subParticles = GetBodyParticles(softBodies[i]);
                for(int j = 0; j < softBodies[i].VerticesNum; j++)
                {
                    softBodies[i].Pos[j] = subParticles[j].pos;
                    softBodies[i].particles[j] = subParticles[j];
                }

                softBodies[i].EndFrame();
            }
        }

        private void ElementConstraintClustering()
        {
            ElementClustering clustering = new(5, elementConstraints.ToArray());

            // Get colored clusters; cluster constains multiple constraints
            var coloredClusterList = clustering.PerformClusteringAndColoring();

            if(coloredClusterList.Count == 0)
            {
                Debug.LogWarning("No cluster");
                return;
            }

            // sort clusters by color
            var sortedclusterList = coloredClusterList.OrderBy(i => i.color).ToList();

            var colorCount = sortedclusterList.GroupBy(n => n.color).OrderBy(g => g.Key).ToList();

            passes = colorCount.Select(g => g.Count()).ToList();
            int[] flatCluster = sortedclusterList.SelectMany(innerList => innerList.constraints).ToArray();
            int[] clusterSizes = sortedclusterList.Select(innerList => innerList.constraints.Count).ToArray();
            int[] clusterStart = clusterSizes.Aggregate(
                new List<int>(),
                (sums, current) =>
                {
                    sums.Add(sums.LastOrDefault());
                    if (sums.Count > 1) sums[sums.Count - 1] += current;
                    return sums;
                }
            ).ToArray();

            //string s = "";
            //for(int i = 0; i < clusterSizes.Length; i++)
            //{
            //    s += clusterSizes[i] + " ";
            //}
            //Debug.Log("Cluster size: " + s);
            
            //s = "";
            //for (int i = 0; i < clusterStart.Length; i++)
            //{
            //    s += clusterStart[i] + " ";
            //}
            //Debug.Log("cluster Star: " + s);

            this.clusters = new NativeArray<int>(flatCluster.Length, Allocator.Persistent);
            this.clusters.CopyFrom(flatCluster);
            this.clusterSizes = new NativeArray<int>(clusterSizes.Length, Allocator.Persistent);
            this.clusterSizes.CopyFrom(clusterSizes);
            this.clusterStarts = new NativeArray<int>(clusterStart.Length, Allocator.Persistent);
            this.clusterStarts.CopyFrom(clusterStart);

            //Debug.Log($"Tets: {TetsNum}");
            //Debug.Log($"cluster total: {clusters.Length}");
            //Debug.Log($"colorCount: {colorCount.Count}");
        }

        #region IJob
        [BurstCompile]
        private struct PreSolveJob : IJobParallelFor
        {
            public REAL dt;
            public REAL3 gravity;

            public NativeArray<SoftBodyParticle> particles;

            public void Execute(int index)
            {
                if (particles[index].invMass == 0f)
                    return;

                SoftBodyParticle p = particles[index];

                REAL3 accel = gravity + p.f_ext * p.invMass;
                p.vel += dt * accel;

                p.prevPos = p.pos;

                p.pos += dt * p.vel;

                particles[index] = p;
            }
        }

        [BurstCompile]
        private struct SolveElementJob : IJobParallelFor
        {
            public REAL dt;
            private REAL mu;
            private REAL lambda;

            public int firstCluster;

            [NativeDisableParallelForRestriction]
            public NativeArray<SoftBodyParticle> particles;

            [ReadOnly]
            public NativeArray<ElementConstraint> elementConstraints;

            [ReadOnly]
            public NativeArray<int> cluster;
            [ReadOnly]
            public NativeArray<int> clusterSizes;
            [ReadOnly]
            public NativeArray<int> clusterStart;


            private REAL3x4 gradients;
            private REAL3x4 gradients_d;
            private REAL3x3 F, dF;

            public void Execute(int index)
            {
                int clusterID = firstCluster + index;
                int size = clusterSizes[clusterID];
                for(int i = 0; i < size; i++)
                {
                    int id = cluster[clusterStart[clusterID] + i];
                    if (elementConstraints[id].active)
                        SolveCoupled(id);
                }
            }

            private REAL3x3 GetDeformationGradient(int tetIndex)
            {
                int id0 = elementConstraints[tetIndex].tet.x;
                int id1 = elementConstraints[tetIndex].tet.y;
                int id2 = elementConstraints[tetIndex].tet.z;
                int id3 = elementConstraints[tetIndex].tet.w;

                REAL3x3 Ds = REAL3x3.zero;
                
                Ds.c0 = particles[id1].pos - particles[id0].pos;
                Ds.c1 = particles[id2].pos - particles[id0].pos;
                Ds.c2 = particles[id3].pos - particles[id0].pos;

                return math.mul(Ds, elementConstraints[tetIndex].invDm);
            }
            private void SolveDeviatoric(int i)
            {
                REAL compliance = 1f / (mu * elementConstraints[i].restVolume);
                REAL3x3 F = GetDeformationGradient(i);
                REAL r_s = math.sqrt(math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2));
                REAL C = r_s;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL r_s_inv = 1f / r_s;

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);
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
                REAL compliance = 1f / (lambda * elementConstraints[i].restVolume);

                REAL3x3 F = GetDeformationGradient(i);
                REAL C = math.determinant(F) - gamma;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);

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
                mu = elementConstraints[i].mu;
                lambda = elementConstraints[i].lambda;

                // vertices index
                int id0 = elementConstraints[i].tet.x;
                int id1 = elementConstraints[i].tet.y;
                int id2 = elementConstraints[i].tet.z;
                int id3 = elementConstraints[i].tet.w;

                REAL compliance_D = 1f / (mu * elementConstraints[i].restVolume);

                REAL gamma = mu / lambda;
                REAL compliance_V = 1f / (lambda * elementConstraints[i].restVolume);

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);

                REAL3x3 F = GetDeformationGradient(i);
                REAL3 f1 = F.c0;
                REAL3 f2 = F.c1;
                REAL3 f3 = F.c2;

                REAL r_s = math.sqrt(math.dot(f1, f1) + math.dot(f2, f2) + math.dot(f3, f3));
                REAL r_s_inv = 1f / r_s;

                REAL C_D = r_s;
                REAL C_V = math.determinant(F) - 1f - gamma;

                if (math.abs(C_D) >= Util.EPSILON && math.abs(C_V) >= Util.EPSILON)
                {
                    // Get particles
                    SoftBodyParticle sbp0 = particles[id0];
                    SoftBodyParticle sbp1 = particles[id1];
                    SoftBodyParticle sbp2 = particles[id2];
                    SoftBodyParticle sbp3 = particles[id3];

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
                    A[0][0] += math.dot(grad0_D, grad0_D) * sbp0.invMass;
                    A[0][0] += math.dot(grad1_D, grad1_D) * sbp1.invMass;
                    A[0][0] += math.dot(grad2_D, grad2_D) * sbp2.invMass;
                    A[0][0] += math.dot(grad3_D, grad3_D) * sbp3.invMass;

                    A[1][0] += math.dot(grad0_D, grad0_V) * sbp0.invMass;
                    A[1][0] += math.dot(grad1_D, grad1_V) * sbp1.invMass;
                    A[1][0] += math.dot(grad2_D, grad2_V) * sbp2.invMass;
                    A[1][0] += math.dot(grad3_D, grad3_V) * sbp3.invMass;

                    A[0][1] = A[1][0];

                    A[1][1] += math.dot(grad0_V, grad0_V) * sbp0.invMass;
                    A[1][1] += math.dot(grad1_V, grad1_V) * sbp1.invMass;
                    A[1][1] += math.dot(grad2_V, grad2_V) * sbp2.invMass;
                    A[1][1] += math.dot(grad3_V, grad3_V) * sbp3.invMass;

                    A += alpha;

                    REAL2 dlambda = Util.LUSolve(A, new REAL2(-C_D, -C_V));

                    sbp0.pos = (REAL3)(sbp0.pos + dlambda[0] * sbp0.invMass * grad0_D + dlambda[1] * sbp0.invMass * grad0_V);
                    sbp1.pos = (REAL3)(sbp1.pos + dlambda[0] * sbp1.invMass * grad1_D + dlambda[1] * sbp1.invMass * grad1_V);
                    sbp2.pos = (REAL3)(sbp2.pos + dlambda[0] * sbp2.invMass * grad2_D + dlambda[1] * sbp2.invMass * grad2_V);
                    sbp3.pos = (REAL3)(sbp3.pos + dlambda[0] * sbp3.invMass * grad3_D + dlambda[1] * sbp3.invMass * grad3_V);

                    particles[id0] = sbp0;
                    particles[id1] = sbp1;
                    particles[id2] = sbp2;
                    particles[id3] = sbp3;
                }
                else
                {
                    SolveDeviatoric(i);
                    SolveVolumetric(i);
                }

            }
            private void ApplyToElement(int i, REAL C, REAL compliance, REAL dt)
            {
                REAL weight = 0f;
                int id0 = elementConstraints[i].tet.x;
                int id1 = elementConstraints[i].tet.y;
                int id2 = elementConstraints[i].tet.z;
                int id3 = elementConstraints[i].tet.w;

                // Get particles
                SoftBodyParticle sbp0 = particles[id0];
                SoftBodyParticle sbp1 = particles[id1];
                SoftBodyParticle sbp2 = particles[id2];
                SoftBodyParticle sbp3 = particles[id3];

                weight += math.lengthsq(gradients_d.c0) * sbp0.invMass;
                weight += math.lengthsq(gradients_d.c1) * sbp1.invMass;
                weight += math.lengthsq(gradients_d.c2) * sbp2.invMass;
                weight += math.lengthsq(gradients_d.c3) * sbp3.invMass;

                if (weight < Util.EPSILON)
                    return;

                REAL h2 = dt * dt;
                REAL alpha = compliance / h2;
                REAL dlambda = -C / (weight + alpha);

                sbp0.pos = (REAL3)(sbp0.pos + dlambda * sbp0.invMass * gradients_d.c0);
                sbp1.pos = (REAL3)(sbp1.pos + dlambda * sbp1.invMass * gradients_d.c1);
                sbp2.pos = (REAL3)(sbp2.pos + dlambda * sbp2.invMass * gradients_d.c2);
                sbp3.pos = (REAL3)(sbp3.pos + dlambda * sbp3.invMass * gradients_d.c3);

                particles[id0] = sbp0;
                particles[id1] = sbp1;
                particles[id2] = sbp2;
                particles[id3] = sbp3;
            }
        }

        [BurstCompile]
        private struct SolveElementJob2 : IJob
        {
            public REAL dt;
            private REAL mu;
            private REAL lambda;

            [NativeDisableParallelForRestriction]
            public NativeArray<SoftBodyParticle> particles;

            [ReadOnly]
            public NativeArray<ElementConstraint> elementConstraints;

            private REAL3x4 gradients;
            private REAL3x4 gradients_d;
            private REAL3x3 F, dF;

            public void Execute()
            {
                for (int index = 0; index < elementConstraints.Length; ++index)
                {
                    if (elementConstraints[index].active)
                        SolveCoupled(index);
                }
            }

            private REAL3x3 GetDeformationGradient(int tetIndex)
            {
                int id0 = elementConstraints[tetIndex].tet.x;
                int id1 = elementConstraints[tetIndex].tet.y;
                int id2 = elementConstraints[tetIndex].tet.z;
                int id3 = elementConstraints[tetIndex].tet.w;

                REAL3x3 Ds = REAL3x3.zero;

                Ds.c0 = particles[id1].pos - particles[id0].pos;
                Ds.c1 = particles[id2].pos - particles[id0].pos;
                Ds.c2 = particles[id3].pos - particles[id0].pos;

                return math.mul(Ds, elementConstraints[tetIndex].invDm);
            }
            private void SolveDeviatoric(int i)
            {
                REAL compliance = 1f / (mu * elementConstraints[i].restVolume);
                REAL3x3 F = GetDeformationGradient(i);
                REAL r_s = math.sqrt(math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2));
                REAL C = r_s;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL r_s_inv = 1f / r_s;

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);
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
                REAL compliance = 1f / (lambda * elementConstraints[i].restVolume);

                REAL3x3 F = GetDeformationGradient(i);
                REAL C = math.determinant(F) - gamma;

                if (math.abs(C) < Util.EPSILON)
                    return;

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);

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
                mu = elementConstraints[i].mu;
                lambda = elementConstraints[i].lambda;

                // vertices index
                int id0 = elementConstraints[i].tet.x;
                int id1 = elementConstraints[i].tet.y;
                int id2 = elementConstraints[i].tet.z;
                int id3 = elementConstraints[i].tet.w;

                REAL compliance_D = 1f / (mu * elementConstraints[i].restVolume);

                REAL gamma = mu / lambda;
                REAL compliance_V = 1f / (lambda * elementConstraints[i].restVolume);

                REAL3x3 QT = math.transpose(elementConstraints[i].invDm);

                REAL3x3 F = GetDeformationGradient(i);
                REAL3 f1 = F.c0;
                REAL3 f2 = F.c1;
                REAL3 f3 = F.c2;

                REAL r_s = math.sqrt(math.dot(f1, f1) + math.dot(f2, f2) + math.dot(f3, f3));
                REAL r_s_inv = 1f / r_s;

                REAL C_D = r_s;
                REAL C_V = math.determinant(F) - 1f - gamma;

                if (math.abs(C_D) >= Util.EPSILON && math.abs(C_V) >= Util.EPSILON)
                {
                    // Get particles
                    SoftBodyParticle sbp0 = particles[id0];
                    SoftBodyParticle sbp1 = particles[id1];
                    SoftBodyParticle sbp2 = particles[id2];
                    SoftBodyParticle sbp3 = particles[id3];

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
                    A[0][0] += math.dot(grad0_D, grad0_D) * sbp0.invMass;
                    A[0][0] += math.dot(grad1_D, grad1_D) * sbp1.invMass;
                    A[0][0] += math.dot(grad2_D, grad2_D) * sbp2.invMass;
                    A[0][0] += math.dot(grad3_D, grad3_D) * sbp3.invMass;

                    A[1][0] += math.dot(grad0_D, grad0_V) * sbp0.invMass;
                    A[1][0] += math.dot(grad1_D, grad1_V) * sbp1.invMass;
                    A[1][0] += math.dot(grad2_D, grad2_V) * sbp2.invMass;
                    A[1][0] += math.dot(grad3_D, grad3_V) * sbp3.invMass;

                    A[0][1] = A[1][0];

                    A[1][1] += math.dot(grad0_V, grad0_V) * sbp0.invMass;
                    A[1][1] += math.dot(grad1_V, grad1_V) * sbp1.invMass;
                    A[1][1] += math.dot(grad2_V, grad2_V) * sbp2.invMass;
                    A[1][1] += math.dot(grad3_V, grad3_V) * sbp3.invMass;

                    A += alpha;

                    REAL2 dlambda = Util.LUSolve(A, new REAL2(-C_D, -C_V));

                    sbp0.pos = (REAL3)(sbp0.pos + dlambda[0] * sbp0.invMass * grad0_D + dlambda[1] * sbp0.invMass * grad0_V);
                    sbp1.pos = (REAL3)(sbp1.pos + dlambda[0] * sbp1.invMass * grad1_D + dlambda[1] * sbp1.invMass * grad1_V);
                    sbp2.pos = (REAL3)(sbp2.pos + dlambda[0] * sbp2.invMass * grad2_D + dlambda[1] * sbp2.invMass * grad2_V);
                    sbp3.pos = (REAL3)(sbp3.pos + dlambda[0] * sbp3.invMass * grad3_D + dlambda[1] * sbp3.invMass * grad3_V);

                    particles[id0] = sbp0;
                    particles[id1] = sbp1;
                    particles[id2] = sbp2;
                    particles[id3] = sbp3;
                }
                else
                {
                    SolveDeviatoric(i);
                    SolveVolumetric(i);
                }

            }
            private void ApplyToElement(int i, REAL C, REAL compliance, REAL dt)
            {
                REAL weight = 0f;
                int id0 = elementConstraints[i].tet.x;
                int id1 = elementConstraints[i].tet.y;
                int id2 = elementConstraints[i].tet.z;
                int id3 = elementConstraints[i].tet.w;

                // Get particles
                SoftBodyParticle sbp0 = particles[id0];
                SoftBodyParticle sbp1 = particles[id1];
                SoftBodyParticle sbp2 = particles[id2];
                SoftBodyParticle sbp3 = particles[id3];

                weight += math.lengthsq(gradients_d.c0) * sbp0.invMass;
                weight += math.lengthsq(gradients_d.c1) * sbp1.invMass;
                weight += math.lengthsq(gradients_d.c2) * sbp2.invMass;
                weight += math.lengthsq(gradients_d.c3) * sbp3.invMass;

                if (weight < Util.EPSILON)
                    return;

                REAL h2 = dt * dt;
                REAL alpha = compliance / h2;
                REAL dlambda = -C / (weight + alpha);

                sbp0.pos = (REAL3)(sbp0.pos + dlambda * sbp0.invMass * gradients_d.c0);
                sbp1.pos = (REAL3)(sbp1.pos + dlambda * sbp1.invMass * gradients_d.c1);
                sbp2.pos = (REAL3)(sbp2.pos + dlambda * sbp2.invMass * gradients_d.c2);
                sbp3.pos = (REAL3)(sbp3.pos + dlambda * sbp3.invMass * gradients_d.c3);

                particles[id0] = sbp0;
                particles[id1] = sbp1;
                particles[id2] = sbp2;
                particles[id3] = sbp3;
            }
        }

        [BurstCompile]
        private struct PostSolveJob : IJobParallelFor
        {
            public REAL dt;

            public NativeArray<SoftBodyParticle> particles;

            public void Execute(int index)
            {
                if (particles[index].invMass == 0f)
                    return;

                SoftBodyParticle p = particles[index];

                //p.pos.y = math.max(0, p.pos.y);
                p.vel = (p.pos - p.prevPos) / dt;

                particles[index] = p;
            }
        }
        #endregion
    }
}
