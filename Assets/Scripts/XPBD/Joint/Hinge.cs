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
        public bool targetOn = false;
        public float targetAngle = 0f;

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
            // Hinge axis align
            float3 A1 = math.rotate(thisBody.Rotation, math.rotate(math.conjugate(q1), axisA));
            float3 A2 = math.rotate(attachedBody.Rotation, math.rotate(math.conjugate(q2), axisA));
            float3 dq_hinge = math.cross(A2, A1);

            NativeArray<AngularConstraintData> angularConstraintDatas = new NativeArray<AngularConstraintData>(1, Allocator.TempJob);
            angularConstraintDatas[0] = new AngularConstraintData(thisBody, attachedBody);
            AngularConstraintJob angularConstraintJob = new AngularConstraintJob
            {
                Datas = angularConstraintDatas,
                dq = dq_hinge,
                angle = 0f,
                compliance = 0f,
                dt = dt
            };
            jobHandle = angularConstraintJob.Schedule();
            jobHandle.Complete();
            
            thisBody.Rotation = angularConstraintDatas[0].q1;
            attachedBody.Rotation = angularConstraintDatas[0].q2;

            // Target angle
            if(targetOn)
            {
                targetAngle %= 360f;
                float targetAngleRadian = math.radians(targetAngle);
                A1 = math.rotate(thisBody.Rotation, math.rotate(math.conjugate(q1), axisA));
                A2 = math.rotate(attachedBody.Rotation, math.rotate(math.conjugate(q2), axisA));
                float3 B1 = math.rotate(thisBody.Rotation, math.rotate(math.conjugate(q1), axisB));
                float3 B2 = math.rotate(attachedBody.Rotation, math.rotate(math.conjugate(q2), axisB));
                float3 bTarget = math.rotate(quaternion.AxisAngle(math.normalize(A1), targetAngleRadian), B1);
                float3 dq_target = math.cross(B2, bTarget);
                angularConstraintDatas[0] = new AngularConstraintData(thisBody, attachedBody);

                angularConstraintJob = new AngularConstraintJob
                {
                    Datas = angularConstraintDatas,
                    dq = dq_target,
                    angle = 0f,
                    compliance = 0f,
                    dt = dt
                };
                jobHandle = angularConstraintJob.Schedule();
                jobHandle.Complete();

                thisBody.Rotation = angularConstraintDatas[0].q1;
                attachedBody.Rotation = angularConstraintDatas[0].q2;
            }


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

        private void OnDrawGizmosSelected()
        {
            if (attachedBody == null)
                return;

            Vector3 R1, R2, a1, a2;
            Vector3 b1 = Vector3.zero, b2 = Vector3.zero;

            if (thisBody == null)
            {
                R1 = transform.position + transform.rotation * anchor;
                R2 = attachedBody.transform.position + attachedBody.transform.rotation * r2;
                a1 = transform.TransformDirection(transform.InverseTransformDirection(axis)) * 1f;
                a2 = attachedBody.transform.TransformDirection(attachedBody.transform.InverseTransformDirection(axis)) * 1f;

                Vector3 B = Vector3.Cross(axis, Vector3.right);
                if(B.magnitude < 1E-6)
                {
                    B = -Vector3.Cross(axis, Vector3.back);
                }
                B.Normalize();

                b1 = transform.TransformDirection(transform.InverseTransformDirection(B)) * 1f;
                b2 = attachedBody.transform.TransformDirection(attachedBody.transform.InverseTransformDirection(B)) * 1f;
            }
            else
            {
                R1 = transform.position + transform.rotation * r1;
                R2 = attachedBody.transform.position + attachedBody.transform.rotation * r2;
                a1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisA)) * 1f;
                a2 = attachedBody.transform.TransformDirection(math.rotate(math.conjugate(q2), axisA)) * 1f;
                b1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisB)) * 1f;
                b2 = attachedBody.transform.TransformDirection(math.rotate(math.conjugate(q2), axisB)) * 1f;
            }

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(R1, 0.1f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(R2, 0.1f);

            
            DrawArrow.ForGizmo(R1, a1, Color.red);
            DrawArrow.ForGizmo(R2, a2, Color.red);
            DrawArrow.ForGizmo(R1, b1, Color.blue);
            DrawArrow.ForGizmo(R2, b2, Color.blue);


        }
    }
}

