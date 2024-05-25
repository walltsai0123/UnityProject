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

        
        private float3 r1, r2;
        private quaternion q1, q2;

        // Aligned axis of each body in local frame
        private float3 axisA1, axisA2;
        // limited axis of each body in local frame
        private float3 axisB1, axisB2;

        public override void SolveConstraint(float dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint2(float dt)
        {
            AngularConstraint angularConstraint = new AngularConstraint(body1, body2);

            float3 axisA1_wc = math.rotate(body1.Rotation, axisA1);
            float3 axisA2_wc = math.rotate(body2.Rotation, axisA2);
            float3 delta_q = math.cross(axisA1_wc, axisA2_wc);
            float d_lambda = angularConstraint.GetDeltaLambda(dt, 0f, 0f, delta_q);
            //lambda += d_lambda;

            if (targetOn)
            {
                float3 n1 = math.rotate(body1.Rotation, axisB1);
                float3 n2 = math.rotate(body2.Rotation, axisB2);
                float3 N = math.rotate(body1.Rotation, axisA1);

                targetAngle %= 360f;
                float targetAngleRadian = math.radians(targetAngle);

                float3 bTarget = math.rotate(quaternion.AxisAngle(N, targetAngleRadian), n1);
                float3 dq_target = math.cross(bTarget, n2);

                AngularConstraint angC2 = new(body1, body2);
                angC2.GetDeltaLambda(dt, 0f, 0f, dq_target);
            }
        }

        private void SolvePositionConstraint2(float dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            float3 p1 = body1.Position + positionConstraint.r1;
            float3 p2 = body2.Position + positionConstraint.r2;
            float3 delta_x = p1 - p2;

            float d_lambda = positionConstraint.GetDeltaLambda(dt, 0f, 0f, delta_x);
            //lambda += d_lambda;
        }

        private void SolveAngularConstraint(float dt)
        {
            // Hinge axis align
            float3 A1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisA1));
            float3 A2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisA1));
            float3 dq_hinge = math.cross(A2, A1);
            //float3 dq_hinge = math.cross(A1, A2);

            SolveAngularConstraint(dt, dq_hinge, 0f, 0f);

            // Target angle
            if(targetOn)
            {
                targetAngle %= 360f;
                float targetAngleRadian = math.radians(targetAngle);
                A1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisA1));
                A2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisA1));
                float3 B1 = math.rotate(body1.Rotation, math.rotate(math.conjugate(q1), axisA2));
                float3 B2 = math.rotate(body2.Rotation, math.rotate(math.conjugate(q2), axisA2));
                float3 bTarget = math.rotate(quaternion.AxisAngle(math.normalize(A1), targetAngleRadian), B1);
                float3 dq_target = math.cross(B2, bTarget);
                //float3 dq_target = math.cross(bTarget, B2);

                SolveAngularConstraint(dt, dq_target, 0f, 0f);
            }
        }
        private void SolvePositionConstraint(float dt)
        {

            float3 R1 = body1.Position + math.rotate(body1.Rotation, r1);
            float3 R2 = body2.Position + math.rotate(body2.Rotation, r2);
            float3 dx = R1 - R2;

            SolvePositionConstraint(dt, r1, r2, dx, 0f, 0f);
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

            // Get local aligned axis
            axisA1 = math.rotate(math.conjugate(body1.Rotation), axis);
            axisA2 = math.rotate(math.conjugate(body2.Rotation), axis);

            // Calculate joint perpendicular unit axes
            float3 axisB = Util.GetPerpendicularVector(axis);
            axisB1 = math.rotate(math.conjugate(body1.Rotation), axisB);
            axisB2 = math.rotate(math.conjugate(body2.Rotation), axisB);
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
                a1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisA1)) * 1f;
                a2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), axisA1)) * 1f;
                b1 = transform.TransformDirection(math.rotate(math.conjugate(q1), axisA2)) * 1f;
                b2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), axisA2)) * 1f;
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

