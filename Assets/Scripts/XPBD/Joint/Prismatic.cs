using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

using Unity.Jobs;


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
    public class Prismatic : Joint
    {
        [SerializeField] Vector3 anchor;
        [SerializeField] Vector3 axis = Vector3.up;

        [SerializeField, Min(0f)]
        private REAL min = 0f;
        [SerializeField, Min(0f)]
        private REAL max = 0f;
        [SerializeField, Min(0f)]
        private REAL compliance = 0f;

        // Local perpendicular axes of body1
        private REAL3 axisA, axisB, axisC;
        private REAL3 r1, r2;
        private quaternion q1, q2;

        Timer timer = new();
        int times = 0;
        float duration = 0;

        public override void SolveConstraint(REAL dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint2(REAL dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            REAL3 dq = 2f * math.mul(Q2, math.conjugate(Q1)).value.xyz;

            AngularConstraint angularConstraint = new(body1, body2);

            angularConstraint.GetDeltaLambda(dt, 0f, 0f, dq);
            angularConstraints[0] = angularConstraint;

            body1.Rotation = angularConstraints[0].q1;
            body2.Rotation = angularConstraints[0].q2;
        }

        private void SolvePositionConstraint2(REAL dt)
        {
            REAL3 R1 = body1.Position + Util.rotate(body1.Rotation, r1);
            REAL3 R2 = body2.Position + Util.rotate(body2.Rotation, r2);
            REAL3 Dr = R1 - R2;
            REAL3 Dx = REAL3.zero;

            REAL3 AxisA = Util.rotate(body1.Rotation, axisA);

            if (max < min)
                max = min;
            // Axis A
            REAL da = math.dot(Dr, AxisA);
            if (da < min)
                Dx += AxisA * (da - min);
            if (da > max)
                Dx += AxisA * (da - max);

            PositionConstraint positionConstraint = new(body1, body2, r1, r2);
            positionConstraint.GetDeltaLambda(dt, compliance, 0f, Dx);
            body1.Position = positionConstraint.x1;
            body2.Position = positionConstraint.x2;
            body1.Rotation = positionConstraint.q1;
            body2.Rotation = positionConstraint.q2;

            ////Axis B
            //REAL db = math.dot(Dr, AxisB);
            //Dx += AxisB * db;

            ////Axis C
            //REAL dc = math.dot(Dr, AxisC);
            //Dx += AxisC * dc;

            R1 = body1.Position + Util.rotate(body1.Rotation, r1);
            R2 = body2.Position + Util.rotate(body2.Rotation, r2);
            Dr = R1 - R2;
            Dx = REAL3.zero;
            AxisA = Util.rotate(body1.Rotation, axisA);

            REAL3 d_bc = Dr - math.dot(Dr, AxisA) * AxisA;
            Dx += d_bc;

            PositionConstraint positionConstraint2 = new(body1, body2, r1, r2);
            positionConstraint2.GetDeltaLambda(dt, 0f, 0f, Dx);

            body1.Position = positionConstraint2.x1;
            body2.Position = positionConstraint2.x2;
            body1.Rotation = positionConstraint2.q1;
            body2.Rotation = positionConstraint2.q2;
        }
        private void Awake()
        {
            body1 = GetComponent<Rigid>();
        }

        protected override void Initialize()
        {
            r1 = (float3)anchor;
            REAL3 Anchor = body1.Position + Util.rotate(body1.Rotation, r1);
            r2 = Util.rotate(math.conjugate(body2.Rotation), Anchor - body2.Position);

            q1 = body1.Rotation;
            q2 = body2.Rotation;

            // Calculate joint perpendicular unit axes
            axisA = math.normalize(axis);
            axisB = Util.GetPerpendicularVector(axisA);
            axisC = math.cross(axisA, axisB);

            axisA = Util.rotate(math.conjugate(body1.Rotation), axisA);
            axisB = Util.rotate(math.conjugate(body1.Rotation), axisB);
            axisC = Util.rotate(math.conjugate(body1.Rotation), axisC);
        }

        private void OnDrawGizmosSelected()
        {
            if (body2 == null)
                return;

            Vector3 R1, R2, dir1, dir2;

            if (body1 == null)
            {
                R1 = transform.position + transform.rotation * anchor;
                R2 = (float3)body2.transform.position + (float3)Util.rotate(body2.transform.rotation, r2);
                dir1 = transform.TransformDirection(transform.InverseTransformDirection(axis)) * 1f;
                dir2 = body2.transform.TransformDirection(body2.transform.InverseTransformDirection(axis)) * 1f;
            }
            else
            {
                R1 = (float3)transform.position + (float3)Util.rotate(transform.rotation, r1);
                R2 = (float3)body2.transform.position + (float3)Util.rotate(body2.transform.rotation, r2);
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
