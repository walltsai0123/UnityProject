using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

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
    public class FixedJoint : Joint
    {
        private REAL3 r1, r2;
        private quaternion q1;
        private quaternion q2;
        private JobHandle jobHandle;

        public override void SolveConstraint(REAL dt)
        {
            SolveAngularConstraint2(dt);
            SolvePositionConstraint2(dt);
        }

        private void SolveAngularConstraint2(REAL dt)
        {
            quaternion Q1 = math.mul(body1.Rotation, math.conjugate(q1));
            quaternion Q2 = math.mul(body2.Rotation, math.conjugate(q2));
            //Q1 = body1.Rotation;
            //Q2 = body2.Rotation;
            REAL3 dq = 2f * math.mul(Q2, math.conjugate(Q1)).value.xyz;

            AngularConstraint angularConstraint = new AngularConstraint(body1, body2);
            REAL dlambda = angularConstraint.GetDeltaLambda(dt, 0f, 0f, dq);

            //Debug.Log("angConstraint " + dlambda * math.normalizesafe(dq));
        }
        private void SolvePositionConstraint2(REAL dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            REAL3 p1 = body1.Position + positionConstraint.r1;
            REAL3 p2 = body2.Position + positionConstraint.r2;
            REAL3 delta_x = p1 - p2;

            REAL dlambda = positionConstraint.GetDeltaLambda(dt, 0f, 0f, delta_x);

            //Debug.Log("posConstraint " + dlambda * math.normalizesafe(delta_x));
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

            r1 = math.rotate(new float4x4(math.conjugate(q1), float3.zero), body2.Position - body1.Position);
            r2 = float3.zero;
        }
    }
}

