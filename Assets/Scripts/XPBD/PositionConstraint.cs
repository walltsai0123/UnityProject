using Unity.Mathematics;
using Unity.Burst;
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
    [BurstCompile]
    public struct PositionConstraint
    {
        // Bodys to be contrained
        //Rigid rigid1;
        //Rigid rigid2;

        // Constraint point on each body in world coordinate
        public REAL3 r1;
        public REAL3 r2;

        public REAL3 x1, x2;
        public quaternion q1, q2;

        public REAL InvMass1;
        public REAL InvMass2;

        // Inverse inertia tensor of each body
        public REAL3x3 rigid1_InertiaInv;
        public REAL3x3 rigid2_InertiaInv;

        public PositionConstraint(Rigid Rigid1, Rigid Rigid2, REAL3 r1_lc, REAL3 r2_lc)
        {
            //this.rigid1 = Rigid1;
            //this.rigid2 = Rigid2;


            q1 = Rigid1.Rotation;
            q2 = Rigid2.Rotation;

            r1 = math.rotate(new float4x4(q1, float3.zero), r1_lc);
            r2 = math.rotate(new float4x4(q2, float3.zero), r2_lc);

            x1 = Rigid1.Position;
            x2 = Rigid2.Position;


            InvMass1 = Rigid1.InvMass;
            InvMass2 = Rigid2.InvMass;

            rigid1_InertiaInv = Rigid1.InertiaInv;
            rigid2_InertiaInv = Rigid2.InertiaInv;
        }

        public REAL GetDeltaLambda(REAL dt, REAL compliance, REAL lambda, REAL3 delta_x)
        {
            REAL C = math.length(delta_x);

            if(C <= Util.EPSILON)
            {
                return 0f;
            }

            REAL3 N = delta_x / C;

            REAL w1 = InvMass1 + math.dot(math.cross(r1, N), math.mul(rigid1_InertiaInv, math.cross(r1, N)));
            REAL w2 = InvMass2 + math.dot(math.cross(r2, N), math.mul(rigid2_InertiaInv, math.cross(r2, N)));

            if(w1 + w2 < Util.EPSILON)
            {
                return 0f;
            }

            REAL alpha = compliance / (dt * dt);
            REAL d_lambda = (-C - alpha * lambda) / (w1 + w2 + alpha);

            // Apply change to rigids
            apply(d_lambda, N);

            return d_lambda;
        }

        private void apply(REAL d_lambda, REAL3 N)
        {
            REAL3 impulse = d_lambda * N;

            // Update position of rigids
            x1 += InvMass1 * impulse;

            x2 -= InvMass2 * impulse;

            // Update rotation of rigids

            REAL3 r1xp = math.mul(rigid1_InertiaInv, math.cross(r1, impulse));
            REAL3 r2xp = math.mul(rigid2_InertiaInv, math.cross(r2, impulse));

            quaternion newQ = new quaternion(0.5f * new float4((float3)r1xp, 0));
            q1 = q1.value + math.mul(newQ, q1).value;
            q1 = math.normalizesafe(q1);

            quaternion newQ2 = new quaternion(0.5f * new float4((float3)r2xp, 0));
            q2 = q2.value - math.mul(newQ2, q2).value;
            q2 = math.normalizesafe(q2);
        }
    }
}

