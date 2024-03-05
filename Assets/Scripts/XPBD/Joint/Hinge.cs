using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class Hinge : Joint
    {
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
            float3 A1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisA));
            float3 A2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisA));
            float3 dq_hinge = math.cross(A2, A1);
            //float3 dq_hinge = math.cross(A1, A2);

            SolveAngularConstraint(dt, dq_hinge, 0f, 0f);

            //NativeArray<AngularConstraintData> angularConstraintDatas = new NativeArray<AngularConstraintData>(1, Allocator.TempJob);
            //angularConstraintDatas[0] = new AngularConstraintData(body1, body2);
            //AngularConstraintJob angularConstraintJob = new AngularConstraintJob
            //{
            //    Datas = angularConstraintDatas,
            //    dq = dq_hinge,
            //    angle = 0f,
            //    compliance = 0f,
            //    dt = dt
            //};
            //jobHandle = angularConstraintJob.Schedule();
            //jobHandle.Complete();
            
            //body1.Rotation = angularConstraintDatas[0].q1;
            //body2.Rotation = angularConstraintDatas[0].q2;

            // Target angle
            if(targetOn)
            {
                targetAngle %= 360f;
                float targetAngleRadian = math.radians(targetAngle);
                A1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisA));
                A2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisA));
                float3 B1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisB));
                float3 B2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisB));
                float3 bTarget = math.rotate(quaternion.AxisAngle(math.normalize(A1), targetAngleRadian), B1);
                float3 dq_target = math.cross(B2, bTarget);
                //float3 dq_target = math.cross(bTarget, B2);

                SolveAngularConstraint(dt, dq_target, 0f, 0f);


                //angularConstraintDatas[0] = new AngularConstraintData(body1, body2);

                //angularConstraintJob = new AngularConstraintJob
                //{
                //    Datas = angularConstraintDatas,
                //    dq = dq_target,
                //    angle = 0f,
                //    compliance = 0f,
                //    dt = dt
                //};
                //jobHandle = angularConstraintJob.Schedule();
                //jobHandle.Complete();

                //body1.Rotation = angularConstraintDatas[0].q1;
                //body2.Rotation = angularConstraintDatas[0].q2;
            }

            //angularConstraintDatas.Dispose();
        }
        private void SolvePositionConstraint(float dt)
        {
            //NativeArray<PositionConstraintData> positionConstraintDatas = new NativeArray<PositionConstraintData>(1, Allocator.TempJob);
            //positionConstraintDatas[0] = new PositionConstraintData(body1, body2, r1, r2);

            float3 R1 = body1.Position + math.rotate(body1.Rotation, r1);
            float3 R2 = body2.Position + math.rotate(body2.Rotation, r2);
            float3 dx = R1 - R2;

            SolvePositionConstraint(dt, r1, r2, dx, 0f, 0f);

            //PositionConstraintJob positionConstraintJob = new PositionConstraintJob
            //{
            //    Datas = positionConstraintDatas,
            //    dx = dx,
            //    dmax = 0f,
            //    compliance = 0f,
            //    dt = dt
            //};
            //jobHandle = positionConstraintJob.Schedule();
            //jobHandle.Complete();

            //body1.Position = positionConstraintDatas[0].x1;
            //body2.Position = positionConstraintDatas[0].x2;
            //body1.Rotation = positionConstraintDatas[0].q1;
            //body2.Rotation = positionConstraintDatas[0].q2;

            //positionConstraintDatas.Dispose();
        }

        private void Awake()
        {
            body1 = GetComponent<Rigid>();
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
            float3 Anchor = body1.Position + math.rotate(body1.Rotation, r1);
            r2 = math.rotate(math.conjugate(body2.Rotation) ,Anchor - body2.Position);

            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = Util.GetPerpendicularVector(axisA);
            //axisB = math.cross(axisA, new float3(1, 0, 0));
            //if(math.length(axisB) < math.EPSILON)
            //{
            //    axisB = -math.cross(axisA, new float3(0, 0, -1));
            //}
            //axisB = math.normalize(axisB);
            axisC = math.cross(axisA, axisB);
        }

        private void OnDrawGizmosSelected()
        {
            if (body2 == null)
                return;

            Vector3 R1, R2, a1, a2;
            Vector3 b1, b2;

            if (body1 == null)
            {
                R1 = transform.position + transform.rotation * anchor;
                R2 = body2.transform.position + body2.transform.rotation * r2;
                a1 = transform.TransformDirection(transform.InverseTransformDirection(axis)) * 1f;
                a2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(axis)) * 1f;

                Vector3 B = Util.GetPerpendicularVector(axis);

                b1 = transform.TransformDirection(transform.InverseTransformDirection(B)) * 1f;
                b2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(B)) * 1f;
            }
            else
            {
                R1 = transform.position + transform.rotation * r1;
                R2 = body2.transform.position + body2.transform.rotation * r2;
                a1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisA)) * 1f;
                a2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), axisA)) * 1f;
                b1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisB)) * 1f;
                b2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), axisB)) * 1f;
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

