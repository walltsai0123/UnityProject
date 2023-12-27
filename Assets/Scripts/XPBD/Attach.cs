using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

namespace XPBD
{
    [RequireComponent(typeof(SoftBody))]
    public class Attach : Constraint
    {
        private SoftBody thisBody;
        public Rigid attachedBody;

        // public int ID1, ID2;

        private struct ParitilcPos
        {
            public int i;
            public float3 pos;
            public ParitilcPos(int index, float3 Pos)
            {
                i = index;
                pos = Pos;
            }
        }
        private struct AttachBodyData
        {
            public float3 position;
            public quaternion rotation;
            public float invMass;
            public float3x3 IBodyInv;
            public AttachBodyData(Rigid attachBody)
            {
                position = attachBody.Position;
                rotation = attachBody.Rotation;
                invMass = attachBody.InvMass;
                IBodyInv = attachBody.InertiaBodyInv;
            }
        }
        private List<ParitilcPos> particlePos;
        private NativeArray<ParitilcPos> particlePosNative;
        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
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

        #region MonoBehaior Methods
        private void Awake()
        {
            thisBody = GetComponent<SoftBody>();
            particlePos = new List<ParitilcPos>();

            Debug.Log("Attach Awake");
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

                float3 clocal = math.rotate(math.conjugate(attachedBody.Rotation), thisBody.Pos[i] - attachedBody.Position);
                particlePos.Add(new ParitilcPos(i, clocal));
            }
            particlePosNative = new NativeArray<ParitilcPos>(particlePos.ToArray(), Allocator.Persistent);
        }

        [BurstCompile]
        private struct AttachJob : IJob
        {
            public NativeArray<ParitilcPos> particlePositions;
            public NativeArray<float3> softbodyPositions;
            public NativeArray<float> softbodyInvMass;
            public NativeArray<AttachBodyData> attachBodyDatas;
            public void Execute()
            {
                AttachBodyData bodyData = attachBodyDatas[0];
                foreach (ParitilcPos pPos in particlePositions)
                {
                    float3 R1 = float3.zero;
                    float3 R2 = math.rotate(bodyData.rotation, pPos.pos);
                    //float3 clocal = math.rotate(math.conjugate(bodyData.rotation), softbodyPositions[pPos.i] - bodyData.position);
                    //float3 dr = clocal - pPos.pos;
                    float3 dr = (softbodyPositions[pPos.i] + R1) - (bodyData.position + R2);
                    float C = math.length(dr);

                    if (C < math.EPSILON)
                        continue;

                    float3 n = math.normalize(dr);
                    float3x3 I2Inv = math.mul(new float3x3(bodyData.rotation), math.mul(bodyData.IBodyInv, new float3x3(math.conjugate(bodyData.rotation))));
                    float3 r2xn = math.cross(R2, n);

                    float w1 = softbodyInvMass[pPos.i];
                    float w2 = bodyData.invMass + math.mul(r2xn, math.mul(I2Inv, r2xn));
                    float alpha = 0.0f;
                    float dlambda = -C / (w1 + w2 + alpha);
                    float3 p = dlambda * n;
                    //p = math.rotate(bodyData.rotation, p);

                    softbodyPositions[pPos.i] += p * w1;
                    bodyData.position -= p * w2;

                    float3 r2xp = math.mul(I2Inv, math.cross(R2, p));
                    quaternion Q2 = bodyData.rotation.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), bodyData.rotation).value;

                    bodyData.rotation = math.normalizesafe(Q2, quaternion.identity);
                }
                attachBodyDatas[0] = bodyData;
            }
        }
    }
    
}

