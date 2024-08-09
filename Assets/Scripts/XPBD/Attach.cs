using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

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
            public REAL3 position;
            public quaternion rotation;
            public REAL invMass;
            public REAL3x3 IBodyInv;
            public AttachBodyData(Rigid attachBody)
            {
                position = attachBody.Position;
                rotation = attachBody.Rotation;
                invMass = attachBody.InvMass;
                IBodyInv = attachBody.InertiaBodyInv;
            }
        }
        private List<ParticlePos> particlePos;
        private NativeArray<ParticlePos> particlePosNative;
        private JobHandle jobHandle;

        public override void SolveConstraint(REAL dt)
        {
            NativeArray<AttachBodyData> attachBodyDatas = new NativeArray<AttachBodyData>(1, Allocator.TempJob);
            attachBodyDatas[0] = new AttachBodyData(attachedBody);
            AttachJob attachJob = new AttachJob
            {
                particlePositions = particlePosNative,
                softbodyPositions = thisBody.Pos,
                softbodyInvMass = thisBody.invMass,
                attachBodyDatas = attachBodyDatas,
            };
            jobHandle = attachJob.Schedule();
            jobHandle.Complete();

            // Update data after job
            thisBody.Pos = attachJob.softbodyPositions;
            attachedBody.Position = attachBodyDatas[0].position;
            attachedBody.Rotation = attachBodyDatas[0].rotation;

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

            //Debug.Log("Attach Awake");
        }
        private void Start()
        {
            //if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            //{
            //    ID1 = attachedBody.ID;
            //    ID2 = thisBody.ID;
            //    BackEnd.AttachRigidSoft(ID1, ID2);
            //}
            Initialized();
            Simulation.get.AddConstraints(this);
        }

        private void OnDestroy()
        {
            if(particlePosNative.IsCreated)
                particlePosNative.Dispose();
        }
        #endregion

        private void Initialized()
        {
            Collider collider = attachedBody.GetComponent<Collider>();
            if(collider == null)
            {
                Debug.LogWarning("Collider null");
                return;
            }
            // Debug.Log(collider);
            for(int i = 0; i < thisBody.VerticesNum; ++i)
            {
                if (!Util.IsInsideCollider(collider, thisBody.Pos[i]))
                    continue;

                REAL3 clocal = math.rotate(new float4x4(math.conjugate(attachedBody.Rotation), float3.zero), thisBody.Pos[i] - attachedBody.Position);
                particlePos.Add(new ParticlePos(i, clocal));
            }
            particlePosNative = new NativeArray<ParticlePos>(particlePos.ToArray(), Allocator.Persistent);
        }

        [BurstCompile]
        private struct AttachJob : IJob
        {
            public NativeArray<ParticlePos> particlePositions;
            public NativeArray<REAL3> softbodyPositions;
            public NativeArray<REAL> softbodyInvMass;
            public NativeArray<AttachBodyData> attachBodyDatas;
            public void Execute()
            {
                AttachBodyData bodyData = attachBodyDatas[0];
                foreach (ParticlePos pPos in particlePositions)
                {
                    REAL3 R1 = REAL3.zero;
                    REAL3 R2 = math.rotate(new float4x4(bodyData.rotation, float3.zero), pPos.pos);
                    //REAL3 clocal = math.rotate(math.conjugate(bodyData.rotation), softbodyPositions[pPos.i] - bodyData.position);
                    //REAL3 dr = clocal - pPos.pos;
                    REAL3 dr = (softbodyPositions[pPos.i] + R1) - (bodyData.position + R2);
                    REAL C = math.length(dr);

                    if (C < math.EPSILON)
                        continue;

                    REAL3 n = math.normalize(dr);
                    REAL3x3 I2Inv = math.mul(new float3x3(bodyData.rotation), math.mul(bodyData.IBodyInv, new float3x3(math.conjugate(bodyData.rotation))));
                    REAL3 r2xn = math.cross(R2, n);

                    REAL w1 = softbodyInvMass[pPos.i];
                    REAL w2 = bodyData.invMass + math.mul(r2xn, math.mul(I2Inv, r2xn));
                    REAL alpha = 0.0f;
                    REAL dlambda = -C / (w1 + w2 + alpha);
                    REAL3 p = dlambda * n;
                    //p = math.rotate(bodyData.rotation, p);

                    softbodyPositions[pPos.i] += p * w1;
                    bodyData.position -= p * w2;

                    REAL3 r2xp = math.mul(I2Inv, math.cross(R2, p));
                    quaternion Q2 = bodyData.rotation.value - math.mul(new quaternion(0.5f * new float4((float3)r2xp, 0f)), bodyData.rotation).value;

                    bodyData.rotation = math.normalizesafe(Q2, quaternion.identity);
                }
                attachBodyDatas[0] = bodyData;
            }
        }
    }
    
}

