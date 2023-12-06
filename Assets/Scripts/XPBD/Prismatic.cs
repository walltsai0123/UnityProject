using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class Prismatic : Constraint
    {
        private Rigid thisBody;
        public Rigid attachedBody;
        public Vector3 anchor;
        public Vector3 axis = Vector3.up;

        [MinAttribute(0f)]
        public float min = 0f;
        [MinAttribute(0f)]
        public float max = 0f;
        [MinAttribute(0f)]
        public float compliance = 0f;

        private float3 axisA, axisB, axisC;
        private float3 r1, r2;
        private quaternion q1, q2;

        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint(dt);
            SolvePositionConstraint(dt);
        }
        private void SolveAngularConstraint(float dt)
        {
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
            float3 Dr = R1 - R2;
            float3 Dx = float3.zero;

            float3 AxisA = math.rotate(thisBody.Rotation, axisA);
            float3 AxisB = math.rotate(thisBody.Rotation, axisB);
            float3 AxisC = math.rotate(thisBody.Rotation, axisC);

            if (max < min)
                max = min;
            // Axis A
            float da = math.dot(Dr, AxisA);
            if (da < min)
                Dx += AxisA * (da - min);
            if (da > max)
                Dx += AxisA * (da - max);

            //Axis B
            float db = math.dot(Dr, AxisB);
            Dx += AxisB * db;

            //Axis C
            float dc = math.dot(Dr, AxisC);
            Dx += AxisC * dc;

            PositionConstraintJob positionConstraintJob = new PositionConstraintJob
            {
                Datas = positionConstraintDatas,
                dx = Dx,
                dmax = 0f,
                compliance = compliance,
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
            Debug.Log("Prismatic Awake");
        }
        void Start()
        {
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        void Initialize()
        {
            r1 = anchor;
            float3 Anchor = thisBody.Position + math.rotate(thisBody.Rotation, r1);
            r2 = math.rotate(math.conjugate(attachedBody.Rotation), Anchor - attachedBody.Position);

            q1 = thisBody.Rotation;
            q2 = attachedBody.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = math.cross(axisA, new float3(1, 0, 0));
            if (math.length(axisB) < math.EPSILON)
            {
                axisB = -math.cross(axisA, new float3(0, 0, -1));
            }
            axisB = math.normalize(axisB);
            axisC = math.cross(axisA, axisB);
        }

        private void OnDrawGizmosSelected()
        {
            if (attachedBody == null)
                return;
            Vector3 R1 = transform.position + transform.rotation * anchor;
            Vector3 R2 = attachedBody.transform.position + attachedBody.transform.rotation * r2;
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(R1, 0.1f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(R2, 0.1f);
        }
    }
}
