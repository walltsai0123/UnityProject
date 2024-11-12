using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using UnityEngine.Assertions;
using Unity.VisualScripting;
using System;
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
    [RequireComponent(typeof(SoftBody))]
    public class Attach : Constraint
    {
        private SoftBody thisBody;
        public Rigid attachedBody;
        private struct ParticlePos
        {
            public int i;
            public REAL3 pos;
            public ParticlePos(int index, REAL3 Pos)
            {
                i = index;
                pos = Pos;
            }
        }
        private struct AttachBodyData
        {
            public readonly bool isFixed;
            public REAL3 position;
            public quaternion rotation;
            public readonly REAL invMass;
            public readonly REAL3x3 IBodyInv;
            public AttachBodyData(Rigid attachBody)
            {
                isFixed = attachBody.isFixed;
                position = attachBody.Position;
                rotation = attachBody.Rotation;
                invMass = attachBody.InvMass;
                IBodyInv = attachBody.InertiaBodyInv;
            }
        }
        private struct CorrectionData
        {
            public REAL3 deltaX;
            public REAL3 deltaOmega;
        }

        private List<ParticlePos> particlePos;
        private NativeArray<ParticlePos> particlePosNative;
        private JobHandle jobHandle;

        public override void SolveConstraint(REAL dt)
        {
            SoftBodySystem sbs = Simulation.get.softBodySystem;

            var particles = sbs.GetBodyParticles(thisBody);

            NativeArray<AttachBodyData> attachBodyDatas = new (1, Allocator.TempJob);
            attachBodyDatas[0] = new AttachBodyData(attachedBody);

            if(Simulation.get.parallelAttach)
            {
                NativeArray<CorrectionData> corrections = new(particlePosNative.Length, Allocator.TempJob);
                AttachJobParallel job = new()
                {
                    particlePositions = particlePosNative,
                    particles = particles,
                    attachBodyDatas = attachBodyDatas,
                    correction = corrections
                };
                jobHandle = job.Schedule(particlePosNative.Length, 1);
                jobHandle.Complete();


                if(!attachedBody.isFixed) {
                    REAL3 omegaSum = 0;
                    foreach (var C in corrections)
                    {
                        attachedBody.Position += C.deltaX;
                        omegaSum += C.deltaOmega;
                    }

                    float3 dir = (float3)math.normalizesafe(omegaSum);
                    float angle = (float)math.length(omegaSum);
                    quaternion deltaRotation = quaternion.AxisAngle(dir, angle);

                    attachedBody.Rotation = math.mul(deltaRotation, attachedBody.Rotation);
                }
                corrections.Dispose();
            }
            else
            {
                AttachJob attachJob = new()
                {
                    particlePositions = particlePosNative,
                    //softbodyPositions = thisBody.Pos,
                    //softbodyInvMass = thisBody.invMass,
                    particles = particles,
                    attachBodyDatas = attachBodyDatas,
                };
                jobHandle = attachJob.Schedule();
                jobHandle.Complete();

                // Update data after job
                attachedBody.Position = attachBodyDatas[0].position;
                attachedBody.Rotation = attachBodyDatas[0].rotation;
            }
            attachBodyDatas.Dispose();

        }

        public override void SolveVelocities(REAL dt)
        {
            //throw new System.NotImplementedException();
        }
        #region MonoBehaior Methods
        private void Awake()
        {
            thisBody = GetComponent<SoftBody>();
            particlePos = new List<ParticlePos>();
        }
        private void OnEnable()
        {
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        private void OnDestroy()
        {
            if(particlePosNative.IsCreated)
                particlePosNative.Dispose();
        }
        #endregion

        protected override void Initialize()
        {
            Collider collider = attachedBody.GetComponent<Collider>();
            if(collider == null)
            {
                Debug.LogWarning("Collider null");
                return;
            }
            // Debug.Log(collider);

            // Record particles inside attachedbody
            bool[] inside = new bool[thisBody.VerticesNum];
            Array.Fill(inside, false);
            for(int i = 0; i < thisBody.VerticesNum; ++i)
            {
                if (!Util.IsInsideCollider(collider, thisBody.Pos[i]))
                    continue;

                REAL3 clocal = math.rotate(new float4x4(math.conjugate(attachedBody.Rotation), float3.zero), thisBody.Pos[i] - attachedBody.Position);
                particlePos.Add(new ParticlePos(i, clocal));
                inside[i] = true;
            }
            particlePosNative = new NativeArray<ParticlePos>(particlePos.ToArray(), Allocator.Persistent);


            thisBody.elementConstraints = thisBody.elementConstraints.Select(x =>
            {
                x.active = !inside[x.tet[0]] || !inside[x.tet[1]] || !inside[x.tet[2]] || !inside[x.tet[3]];
                return x;
            }).ToList();
        }

        [BurstCompile]
        private struct AttachJob : IJob
        {
            public NativeArray<ParticlePos> particlePositions;
            public NativeSlice<SoftBodyParticle> particles;
            public NativeArray<AttachBodyData> attachBodyDatas;
            public void Execute()
            {
                AttachBodyData bodyData = attachBodyDatas[0];
                foreach (ParticlePos pPos in particlePositions)
                {
                    SoftBodyParticle particle = particles[pPos.i];
                    REAL3 R1 = REAL3.zero;
                    REAL3 R2 = math.rotate(new float4x4(bodyData.rotation, float3.zero), pPos.pos);
                    //REAL3 clocal = math.rotate(math.conjugate(bodyData.rotation), particle.pos - bodyData.position);
                    //REAL3 dr = clocal - pPos.pos;
                    REAL3 dr = (particle.pos + R1) - (bodyData.position + R2);
                    REAL C = math.length(dr);

                    if (C < math.EPSILON)
                        continue;

                    REAL3 n = math.normalize(dr);
                    REAL3x3 I2Inv = math.mul(new float3x3(bodyData.rotation), math.mul(bodyData.IBodyInv, new float3x3(math.conjugate(bodyData.rotation))));
                    REAL3 r2xn = math.cross(R2, n);

                    REAL w1 = particle.invMass;
                    REAL w2 = bodyData.invMass + math.mul(r2xn, math.mul(I2Inv, r2xn));
                    REAL alpha = 0.0f;
                    REAL dlambda = -C / (w1 + w2 + alpha);
                    REAL3 p = dlambda * n;
                    //p = math.rotate(bodyData.rotation, p);

                    particle.pos += p * w1;
                    bodyData.position -= p * bodyData.invMass;

                    REAL3 r2xp = math.mul(I2Inv, math.cross(R2, p));
                    quaternion Q2 = bodyData.rotation.value - math.mul(new quaternion(0.5f * new float4((float3)r2xp, 0f)), bodyData.rotation).value;

                    bodyData.rotation = math.normalizesafe(Q2, quaternion.identity);

                    particles[pPos.i] = particle;
                }
                if (!attachBodyDatas[0].isFixed)
                    attachBodyDatas[0] = bodyData;
            }
        }

        [BurstCompile]
        private struct AttachJobParallel : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeSlice<SoftBodyParticle> particles;

            public NativeArray<CorrectionData> correction;

            [ReadOnly]
            public NativeArray<ParticlePos> particlePositions;
            [ReadOnly]
            public NativeArray<AttachBodyData> attachBodyDatas;
            public void Execute(int index)
            {
                AttachBodyData bodyData = attachBodyDatas[0];

                ParticlePos pPos = particlePositions[index];
                SoftBodyParticle particle = particles[pPos.i];

                int nc = particlePositions.Length;

                REAL3 R1 = 0;
                REAL3 R2 = math.rotate(new float4x4(bodyData.rotation, float3.zero), pPos.pos);
                //REAL3 clocal = math.rotate(math.conjugate(bodyData.rotation), particle.pos - bodyData.position);
                //REAL3 dr = clocal - pPos.pos;
                REAL3 dr = (particle.pos + R1) - (bodyData.position + R2);
                REAL C = math.length(dr);

                if (C < math.EPSILON)
                    return;
    
                REAL3 n = math.normalize(dr);
                REAL3x3 I2Inv = math.mul(new float3x3(bodyData.rotation), math.mul(bodyData.IBodyInv, new float3x3(math.conjugate(bodyData.rotation))));
                REAL3 r2xn = math.cross(R2, n);

                REAL w1 = particle.invMass;
                REAL w2 = bodyData.invMass * nc + math.mul(r2xn, math.mul(I2Inv, r2xn));
                REAL alpha = 0.0f;
                REAL dlambda = -C / (w1 + w2 + alpha);
                REAL3 p = dlambda * n;
                //p = math.rotate(bodyData.rotation, p);

                particle.pos += p * w1;
                //bodyData.position -= p * w2;

                REAL3 r2xp = math.mul(I2Inv, math.cross(R2, p));
                //quaternion Q2 = bodyData.rotation.value - math.mul(new quaternion(0.5f * new float4((float3)r2xp, 0f)), bodyData.rotation).value;
                //
                //bodyData.rotation = math.normalizesafe(Q2, quaternion.identity);

                //bodyData.deltaOmega = -r2xp;

                particles[pPos.i] = particle;
                correction[index] = new()
                {
                    deltaX = -p * bodyData.invMass,
                    deltaOmega = -r2xp,
                };
            }
        }
    }
    
}

