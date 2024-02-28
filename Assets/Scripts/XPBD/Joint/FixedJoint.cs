using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class FixedJoint : Constraint
    {
        private Rigid thisBody;
        public Rigid attachedBody;

        // public int ID1, ID2;

        //private float3 anchor;
        private float3 r1, r2;
        private quaternion q1;
        private quaternion q2;
        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint(dt);
            SolvePositionConstraint(dt);
        }
        private void SolveAngularConstraint(float dt)
        {
            //quaternion Q1 = math.mul(math.conjugate(q1), thisBody.Rotation);
            //quaternion Q2 = math.mul(math.conjugate(q2), attachedBody.Rotation);
            quaternion Q1 = math.mul(thisBody.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(attachedBody.Rotation, math.conjugate(q2));
            float3 dq = 2f * math.mul(Q1, math.conjugate(Q2)).value.xyz;

            NativeArray<AngularConstraintData> angularConstraintDatas = new NativeArray<AngularConstraintData>(1, Allocator.TempJob);
            angularConstraintDatas[0] = new AngularConstraintData(thisBody, attachedBody);
            AngularConstraintJob angularConstraintJob = new AngularConstraintJob
            {
                Datas = angularConstraintDatas,
                dq = dq,
                angle = 0f,
                compliance = 0f,
                dt = dt
            };
            jobHandle = angularConstraintJob.Schedule();
            jobHandle.Complete();

            //thisBody.Rotation = math.mul(b1q0, angularConstraintDatas[0].q1);
            //attachedBody.Rotation = math.mul(b2q0, angularConstraintDatas[0].q2);
            thisBody.Rotation = angularConstraintDatas[0].q1;
            attachedBody.Rotation = angularConstraintDatas[0].q2;

            angularConstraintDatas.Dispose();
        }

        private void SolvePositionConstraint(float dt)
        {
            NativeArray<PositionConstraintData> positionConstraintDatas = new NativeArray<PositionConstraintData>(1, Allocator.TempJob);
            positionConstraintDatas[0] = new PositionConstraintData(thisBody, attachedBody, r1, r2);

            float3 R1 = thisBody.Position + math.rotate(thisBody.Rotation, r1);
            float3 R2 = attachedBody.Position + math.rotate(attachedBody.Rotation, r2);
            float3 dx = R1 - R2;

            PositionConstraintJob positionConstraintJob = new PositionConstraintJob
            {
                Datas = positionConstraintDatas,
                dx = dx,
                dmax = 0f,
                compliance = 0f,
                dt = dt
            };
            jobHandle = positionConstraintJob.Schedule();
            jobHandle.Complete();

            thisBody.Position = positionConstraintDatas[0].x1;
            attachedBody.Position = positionConstraintDatas[0].x2;
            thisBody.Rotation = positionConstraintDatas[0].q1;
            attachedBody.Rotation = positionConstraintDatas[0].q2;

            positionConstraintDatas.Dispose();
        }

        private void Awake()
        {
            thisBody = GetComponent<Rigid>();
            Debug.Log("FixedJoint Awake");
        }
        void Start()
        {
            //if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            //{
            //    ID1 = thisBody.ID;
            //    ID2 = attachedBody.ID;

            //    BackEnd.AddFixedJoint(ID1, ID2);
            //}
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        void Initialize()
        {
            q1 = thisBody.Rotation;
            q2 = attachedBody.Rotation;

            // anchor = math.rotate(math.conjugate(b1q0), attachedBody.Position - thisBody.Position);
            r1 = math.rotate(math.conjugate(q1), attachedBody.Position - thisBody.Position);
            r2 = float3.zero;
        }
    }
}

