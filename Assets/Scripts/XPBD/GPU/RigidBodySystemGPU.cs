using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

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
    public class RigidBodySystemGPU : MonoBehaviour
    {
        List<Rigid> rigids;

        [SerializeField] ComputeShader rigidCS;
        public ComputeBuffer rigidsBuffer;

        RigidBodyData[] rigidBodyDatas;

        public struct RigidBodyData
        {
            public int isFixed;
            public REAL3 position;
            public REAL3 prevPos;
            public quaternion rotation;
            public quaternion prevRot;
            public REAL3 vel;
            public REAL3 omega;
            public REAL invMass;
            public REAL3x3 IBody;
            public REAL3x3 IBodyInv;
            public RigidBodyData(Rigid body)
            {
                isFixed = body.isFixed ? 1 : 0;
                position = body.Position;
                prevPos = body.prevPos;
                rotation = body.Rotation;
                prevRot = body.prevRot;
                vel = body.vel;
                omega = body.omega;
                invMass = body.InvMass;
                IBody = body.InertiaBody;
                IBodyInv = body.InertiaBodyInv;
            }
        }

        struct RigidBodyKernel
        {
            public const int PreSolve = 0;
            public const int PostSolve = 1;
        };

        public int GetRigidIndex(Rigid body)
        {
            return rigids.IndexOf(body);
        }
        public void CollectRigidBodies(List<Rigid> bodies)
        {
            rigids = bodies;

            if (rigids.Count == 0)
                return;

            InitComputeBuffers();
        }

        public void PreSolve(REAL dt, REAL3 gravity)
        {
            if (rigids.Count == 0) return;

            rigidCS.SetFloat("dt", (float)dt);
            rigidCS.SetInt("rigidsNum", rigids.Count);
            rigidCS.SetVector("gravity", new float4((float3)gravity, 0f));
            ComputeHelper.Dispatch(rigidCS, rigids.Count, kernelIndex: RigidBodyKernel.PreSolve);
        }

        public void PostSolve(REAL dt)
        {
            if (rigids.Count == 0) return;

            rigidCS.SetFloat("dt", (float)dt);
            rigidCS.SetInt("rigidsNum", rigids.Count);
            ComputeHelper.Dispatch(rigidCS, rigids.Count, kernelIndex: RigidBodyKernel.PostSolve);
        }

        public void EndFrame()
        {
            if (rigids.Count == 0) return;

            rigidsBuffer.GetData(rigidBodyDatas);

            for (int i = 0; i < rigids.Count; i++)
            {
                rigids[i].Position = rigidBodyDatas[i].position;
                rigids[i].Rotation = rigidBodyDatas[i].rotation;
                rigids[i].EndFrame();
            }
        }
        private void InitComputeBuffers()
        {
            rigidCS = Instantiate(Resources.Load<ComputeShader>("RigidBody"));

            rigidBodyDatas = new RigidBodyData[rigids.Count];

            for(int i = 0; i < rigidBodyDatas.Length; i++)
            {
                rigidBodyDatas[i] = new RigidBodyData(rigids[i]);
            }

            rigidsBuffer = ComputeHelper.CreateStructuredBuffer(rigidBodyDatas);
            ComputeHelper.SetBuffer(rigidCS, rigidsBuffer, "rigidBodies", RigidBodyKernel.PreSolve, RigidBodyKernel.PostSolve);
        }
    }
}

