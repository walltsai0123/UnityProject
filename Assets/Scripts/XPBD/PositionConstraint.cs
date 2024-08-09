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
    public struct PositionConstraint
    {
        // Bodys to be contrained
        Rigid rigid1;
        Rigid rigid2;

        // Constraint point on each body in world coordinate
        public REAL3 r1;
        public REAL3 r2;

        // Inverse inertia tensor of each body
        REAL3x3 rigid1_InertiaInv;
        REAL3x3 rigid2_InertiaInv;

        public PositionConstraint(Rigid Rigid1, Rigid Rigid2, REAL3 r1_lc, REAL3 r2_lc)
        {
            this.rigid1 = Rigid1;
            this.rigid2 = Rigid2;

            r1 = math.rotate(new float4x4(rigid1.Rotation, float3.zero), r1_lc);
            r2 = math.rotate(new float4x4(rigid2.Rotation, float3.zero), r2_lc);

            rigid1_InertiaInv = rigid1.InertiaInv;
            rigid2_InertiaInv = rigid2.InertiaInv;
        }

        public REAL GetDeltaLambda(REAL dt, REAL compliance, REAL lambda, REAL3 delta_x)
        {
            REAL C = math.length(delta_x);

            if(C <= Util.EPSILON)
            {
                return 0f;
            }

            REAL3 N = delta_x / C;

            REAL w1 = rigid1.InvMass + math.dot(math.cross(r1, N), math.mul(rigid1_InertiaInv, math.cross(r1, N)));
            REAL w2 = rigid2.InvMass + math.dot(math.cross(r2, N), math.mul(rigid2_InertiaInv, math.cross(r2, N)));

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
            if(!rigid1.isFixed)
                rigid1.Position += rigid1.InvMass * impulse;

            if(!rigid2.isFixed)
                rigid2.Position += -rigid2.InvMass * impulse;

            // Update rotation of rigids

            REAL3 r1xp = math.mul(rigid1_InertiaInv, math.cross(r1, impulse));
            REAL3 r2xp = math.mul(rigid2_InertiaInv, math.cross(r2, impulse));

            if (!rigid1.isFixed)
            {
                quaternion newQ = new quaternion(0.5f * new float4((float3)r1xp, 0));
                rigid1.Rotation = rigid1.Rotation.value + math.mul(newQ, rigid1.Rotation).value;
                rigid1.Rotation = math.normalizesafe(rigid1.Rotation);
            }

            if (!rigid2.isFixed)
            {
                quaternion newQ = new quaternion(0.5f * new float4((float3)r2xp, 0));
                rigid2.Rotation = rigid2.Rotation.value - math.mul(newQ, rigid2.Rotation).value;
                rigid2.Rotation = math.normalizesafe(rigid2.Rotation);
            }
        }
    }
}

