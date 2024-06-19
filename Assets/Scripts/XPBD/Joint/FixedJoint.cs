using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class FixedJoint : Joint
    {
        private float3 r1, r2;
        private quaternion q1;
        private quaternion q2;
        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint2(float dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            //Q1 = body1.Rotation;
            //Q2 = body2.Rotation;
            float3 dq = 2f * math.mul(Q2, math.conjugate(Q1)).value.xyz;

            AngularConstraint angularConstraint = new AngularConstraint(body1, body2);
            float dlambda = angularConstraint.GetDeltaLambda(dt, 0f, 0f, dq);

            //Debug.Log("angConstraint " + dlambda * math.normalizesafe(dq));
        }
        private void SolvePositionConstraint2(float dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            float3 p1 = body1.Position + positionConstraint.r1;
            float3 p2 = body2.Position + positionConstraint.r2;
            float3 delta_x = p1 - p2;

            float dlambda = positionConstraint.GetDeltaLambda(dt, 0f, 0f, delta_x);

            //Debug.Log("posConstraint " + dlambda * math.normalizesafe(delta_x));
        }
        private void SolveAngularConstraint(float dt)
        {
            //quaternion Q1 = math.mul(math.conjugate(q1), thisBody.Rotation);
            //quaternion Q2 = math.mul(math.conjugate(q2), attachedBody.Rotation);
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            float3 dq = 2f * math.mul(Q1, math.conjugate(Q2)).value.xyz;

            NativeArray<AngularConstraintData> angularConstraintDatas = new NativeArray<AngularConstraintData>(1, Allocator.TempJob);
            angularConstraintDatas[0] = new AngularConstraintData(body1, body2);
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
            body1.Rotation = angularConstraintDatas[0].q1;
            body2.Rotation = angularConstraintDatas[0].q2;

            angularConstraintDatas.Dispose();
        }

        private void SolvePositionConstraint(float dt)
        {
            NativeArray<PositionConstraintData> positionConstraintDatas = new NativeArray<PositionConstraintData>(1, Allocator.TempJob);
            positionConstraintDatas[0] = new PositionConstraintData(body1, body2, r1, r2);

            float3 R1 = body1.Position + math.rotate(body1.Rotation, r1);
            float3 R2 = body2.Position + math.rotate(body2.Rotation, r2);
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

            body1.Position = positionConstraintDatas[0].x1;
            body2.Position = positionConstraintDatas[0].x2;
            body1.Rotation = positionConstraintDatas[0].q1;
            body2.Rotation = positionConstraintDatas[0].q2;

            positionConstraintDatas.Dispose();
        }

        private void Awake()
        {
            body1 = GetComponent<Rigid>();
        }
        void Start()
        {
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        void Initialize()
        {
            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // anchor = math.rotate(math.conjugate(b1q0), attachedBody.Position - thisBody.Position);
            r1 = math.rotate(math.conjugate(q1), body2.Position - body1.Position);
            r2 = float3.zero;
        }
    }
}

