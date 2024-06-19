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
        [SerializeField] Vector3 anchor;
        [SerializeField] Vector3 axis = Vector3.up;

        [SerializeField, Min(0f)]
        private float min = 0f;
        [SerializeField, Min(0f)]
        private float max = 0f;
        [SerializeField, Min(0f)]
        private float compliance = 0f;

        // Local perpendicular axes of body1
        private float3 axisA, axisB, axisC;
        private float3 r1, r2;
        private quaternion q1, q2;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint(float dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            float3 dq = 2f * math.mul(Q1, math.conjugate(Q2)).value.xyz;

            SolveAngularConstraint(dt, dq, 0f, 0f);
        }
        private void SolvePositionConstraint(float dt)
        {
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
        }

        private void SolveAngularConstraint2(float dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            float3 dq = 2f * math.mul(Q2, math.conjugate(Q1)).value.xyz;

            AngularConstraint angularConstraint = new AngularConstraint(body1, body2);
            angularConstraint.GetDeltaLambda(dt, 0f, 0f, dq);
        }

        private void SolvePositionConstraint2(float dt)
        {
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

            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);
            positionConstraint.GetDeltaLambda(dt, compliance, 0f, Dx);

            ////Axis B
            //float db = math.dot(Dr, AxisB);
            //Dx += AxisB * db;

            ////Axis C
            //float dc = math.dot(Dr, AxisC);
            //Dx += AxisC * dc;

            R1 = body1.Position + math.rotate(body1.Rotation, r1);
            R2 = body2.Position + math.rotate(body2.Rotation, r2);
            Dr = R1 - R2;
            Dx = float3.zero;
            AxisA = math.rotate(body1.Rotation, axisA);

            float3 d_bc = Dr - math.dot(Dr, AxisA) * AxisA;
            Dx += d_bc;

            PositionConstraint positionConstraint2 = new PositionConstraint(body1, body2, r1, r2);
            positionConstraint2.GetDeltaLambda(dt, 0f, 0f, Dx);
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
            r1 = anchor;
            float3 Anchor = body1.Position + math.rotate(body1.Rotation, r1);
            r2 = math.rotate(math.conjugate(body2.Rotation), Anchor - body2.Position);

            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = Util.GetPerpendicularVector(axisA);
            axisC = math.cross(axisA, axisB);

            axisA = math.rotate(math.conjugate(body1.Rotation), axisA);
            axisB = math.rotate(math.conjugate(body1.Rotation), axisB);
            axisC = math.rotate(math.conjugate(body1.Rotation), axisC);
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
