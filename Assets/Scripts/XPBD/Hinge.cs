using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class Hinge : Constraint
    {
        private Rigid thisBody;
        public Rigid attachedBody;
        public Vector3 anchor;
        public Vector3 axis = Vector3.right;
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
            float3 A1 = math.rotate(thisBody.Rotation, math.rotate(q1, axisA));
            float3 A2 = math.rotate(attachedBody.Rotation, math.rotate(q2, axisA));
            float3 dq = math.cross(A1, A2);

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
            Debug.Log("Hinge Awake");
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
            r2 = math.rotate(math.conjugate(attachedBody.Rotation) ,Anchor - attachedBody.Position);

            q1 = thisBody.Rotation;
            q2 = attachedBody.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = math.cross(axisA, new float3(1, 0, 0));
            if(math.length(axisB) < math.EPSILON)
            {
                axisB = -math.cross(axisA, new float3(0, 0, -1));
            }
            axisB = math.normalize(axisB);
            axisC = math.cross(axisA, axisB);
        }
    }
}

