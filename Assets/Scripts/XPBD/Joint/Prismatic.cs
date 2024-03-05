using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class Prismatic : Joint
    {
        public Vector3 anchor;
        public Vector3 axis = Vector3.up;

        [SerializeField, Min(0f)]
        private float min = 0f;
        [SerializeField, Min(0f)]
        private float max = 0f;
        [SerializeField, Min(0f)]
        private float compliance = 0f;

        [SerializeField, Range(0f, 1f)]
        private float linearDamping = 0f;

        [SerializeField, Range(0f, 1f)]
        private float angularDamping = 0f;

        private float3 axisA, axisB, axisC;
        private float3 r1, r2;
        private quaternion q1, q2;

        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint(dt);
            SolvePositionConstraint(dt);
        }
        public override void SolveVelocities(float dt)
        {
            float3 dv = (body2.vel - body1.vel) * math.min(linearDamping, 1);
            float3 domega = (body2.omega - body1.omega) * math.min(angularDamping, 1);

            // linear part
            float3 p = dv / (body1.InvMass + body2.InvMass);
            body1.vel += p * body1.InvMass;
            body2.vel -= p * body2.InvMass;

            // angular part
            float3 n = math.normalizesafe(domega, float3.zero);
            float w1 = math.mul(n, math.mul(body1.InertiaInv, n));
            float w2 = math.mul(n, math.mul(body2.InertiaInv, n));
            p = domega / (w1 + w2);
            body1.omega += math.mul(body1.InertiaInv, p);
            body2.omega -= math.mul(body2.InertiaInv, p);

        }
        private void SolveAngularConstraint(float dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            float3 dq = 2f * math.mul(Q1, math.conjugate(Q2)).value.xyz;

            SolveAngularConstraint(dt, dq, 0f, 0f);

            //NativeArray<AngularConstraintData> angularConstraintDatas = new NativeArray<AngularConstraintData>(1, Allocator.TempJob);
            //angularConstraintDatas[0] = new AngularConstraintData(body1, body2);
            //AngularConstraintJob angularConstraintJob = new AngularConstraintJob
            //{
            //    Datas = angularConstraintDatas,
            //    dq = dq,
            //    angle = 0f,
            //    compliance = 0f,
            //    dt = dt
            //};
            //jobHandle = angularConstraintJob.Schedule();
            //jobHandle.Complete();

            //body1.Rotation = angularConstraintDatas[0].q1;
            //body2.Rotation = angularConstraintDatas[0].q2;

            //angularConstraintDatas.Dispose();
        }
        private void SolvePositionConstraint(float dt)
        {
            //NativeArray<PositionConstraintData> positionConstraintDatas = new NativeArray<PositionConstraintData>(1, Allocator.TempJob);
            //positionConstraintDatas[0] = new PositionConstraintData(body1, body2, r1, r2);

            float3 R1 = body1.Position + math.rotate(body1.Rotation, r1);
            float3 R2 = body2.Position + math.rotate(body2.Rotation, r2);
            float3 Dr = R1 - R2;
            float3 Dx = float3.zero;

            float3 AxisA = math.rotate(body1.Rotation, axisA);
            float3 AxisB = math.rotate(body1.Rotation, axisB);
            float3 AxisC = math.rotate(body1.Rotation, axisC);

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

            SolvePositionConstraint(dt, r1, r2, Dx, 0f, compliance);

            //PositionConstraintJob positionConstraintJob = new PositionConstraintJob
            //{
            //    Datas = positionConstraintDatas,
            //    dx = Dx,
            //    dmax = 0f,
            //    compliance = compliance,
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
            //Debug.Log("Prismatic Awake");
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
            r2 = math.rotate(math.conjugate(body2.Rotation), Anchor - body2.Position);

            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = Util.GetPerpendicularVector(axisA);
            //axisB = math.cross(axisA, new float3(1, 0, 0));
            //if (math.length(axisB) < math.EPSILON)
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

            Vector3 R1, R2, dir1, dir2;

            if (body1 == null)
            {
                R1 = transform.position + transform.rotation * anchor;
                R2 = body2.transform.position + body2.transform.rotation * r2;
                dir1 = transform.TransformDirection(transform.InverseTransformDirection(axis)) * 1f;
                dir2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(axis)) * 1f;
            }
            else
            {
                R1 = transform.position + transform.rotation * r1;
                R2 = body2.transform.position + body2.transform.rotation * r2;
                dir1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axis)) * 1f;
                dir2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), axis)) * 1f;
            }

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(R1, 0.1f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(R2, 0.1f);

            DrawArrow.ForGizmo(R1, dir1, Color.red);
            DrawArrow.ForGizmo(R2, dir2, Color.blue);
        }
    }
}
