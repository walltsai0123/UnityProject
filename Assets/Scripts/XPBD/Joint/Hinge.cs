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

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class Hinge : Joint
    {
        public Vector3 anchor;
        public Vector3 axis = Vector3.right;
        public bool targetOn = false;
        public REAL targetAngle = 0;

        
        private REAL3 r1, r2;
        private quaternion q1, q2;

        // Aligned axis of each body in local frame
        private REAL3 axisA1, axisA2;
        // limited axis of each body in local frame
        private REAL3 axisB1, axisB2;

        public override void SolveConstraint(REAL dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint2(REAL dt)
        {
            AngularConstraint angularConstraint = new AngularConstraint(body1, body2);

            REAL3 axisA1_wc = math.rotate(new float4x4(body1.Rotation, 0), axisA1);
            REAL3 axisA2_wc = math.rotate(new float4x4(body2.Rotation, 0), axisA2);
            REAL3 delta_q = math.cross(axisA1_wc, axisA2_wc);
            REAL d_lambda = angularConstraint.GetDeltaLambda(dt, 0f, 0f, delta_q);
            //lambda += d_lambda;

            if (targetOn)
            {
                REAL3 n1 = math.rotate(new float4x4(body1.Rotation, 0), axisB1);
                REAL3 n2 = math.rotate(new float4x4(body2.Rotation, 0), axisB2);
                REAL3 N = math.rotate(new float4x4(body1.Rotation, 0), axisA1);

                targetAngle %= 360f;
                REAL targetAngleRadian = math.radians(targetAngle);

                quaternion q = quaternion.AxisAngle((float3)N, (float)targetAngleRadian);
                REAL3 bTarget = math.rotate(new float4x4(q, float3.zero), n1);
                REAL3 dq_target = math.cross(bTarget, n2);

                AngularConstraint angC2 = new(body1, body2);
                angC2.GetDeltaLambda(dt, 0f, 0f, dq_target);
            }
        }

        private void SolvePositionConstraint2(REAL dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            REAL3 p1 = body1.Position + positionConstraint.r1;
            REAL3 p2 = body2.Position + positionConstraint.r2;
            REAL3 delta_x = p1 - p2;

            REAL d_lambda = positionConstraint.GetDeltaLambda(dt, 0f, 0f, delta_x);
            //lambda += d_lambda;
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
            r1 = (float3)anchor;
            REAL3 Anchor = body1.Position + math.rotate(new float4x4(body1.Rotation, 0), r1);
            r2 = math.rotate(new float4x4(math.conjugate(body2.Rotation), 0), Anchor - body2.Position);

            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // Get local aligned axis
            axisA1 = math.rotate(math.conjugate(body1.Rotation), axis);
            axisA2 = math.rotate(math.conjugate(body2.Rotation), axis);

            // Calculate joint perpendicular unit axes
            REAL3 axisB = Util.GetPerpendicularVector((float3)axis);
            axisB1 = Util.rotate(math.conjugate(body1.Rotation), axisB);
            axisB2 = Util.rotate(math.conjugate(body2.Rotation), axisB);
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
                R2 = body2.transform.position + body2.transform.rotation * (float3)r2;
                a1 = transform.TransformDirection(transform.InverseTransformDirection(axis)) * 1f;
                a2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(axis)) * 1f;

                Vector3 B = (float3)Util.GetPerpendicularVector((float3)axis);

                b1 = transform.TransformDirection(transform.InverseTransformDirection(B)) * 1f;
                b2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(B)) * 1f;
            }
            else
            {
                R1 = transform.position + transform.rotation * (float3)r1;
                R2 = body2.transform.position + body2.transform.rotation * (float3)r2;
                a1 = transform.TransformDirection(math.rotate(math.conjugate(q1), (float3)axisA1)) * 1f;
                a2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), (float3)axisA1)) * 1f;
                b1 = transform.TransformDirection(math.rotate(math.conjugate(q1), (float3)axisA2)) * 1f;
                b2 = body2.transform.TransformDirection(math.rotate(math.conjugate(q2), (float3)axisA2)) * 1f;
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

