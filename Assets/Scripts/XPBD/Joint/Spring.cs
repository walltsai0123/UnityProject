using System;
using Unity.Mathematics;
using UnityEngine;

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
    public class Spring : Joint
    {
        public Vector3 anchor;
        public Vector3 distance;

        [SerializeField, Min(0f)]
        private float compliance = 0f;

        private REAL3 r1, r2;
        public override void SolveConstraint(REAL dt)
        {
            SolvePositionConstraint(dt);
        }


        private void SolvePositionConstraint(REAL dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            REAL3 p1 = body1.Position + positionConstraint.r1;
            REAL3 p2 = body2.Position + positionConstraint.r2;
            REAL3 dist = (float3)distance;
            REAL3 delta_x = p1 - p2 - dist;

            REAL d_lambda = positionConstraint.GetDeltaLambda(dt, compliance, 0f, delta_x);
        }

        private void SolveAngularConstraint(float dt)
        {
            throw new NotImplementedException();
        }

        void Awake()
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
            REAL3 Anchor = body1.Position + math.rotate(new float4x4(body1.Rotation, float3.zero), r1);
            r2 = math.rotate(new float4x4(math.conjugate(body2.Rotation), float3.zero), Anchor - body2.Position);
        }

        private void OnDrawGizmosSelected()
        {
            if (body2 == null)
                return;

            Vector3 R1 = transform.position + transform.rotation * anchor;
            Vector3 R2 = body2.transform.position + body2.transform.rotation * (float3)r2;


            Gizmos.color = Color.green;
            Gizmos.DrawSphere(R1, 0.1f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(R2, 0.1f);

            Gizmos.DrawLine(R1, R2);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(R1, R1 - distance);
        }
    }
}
