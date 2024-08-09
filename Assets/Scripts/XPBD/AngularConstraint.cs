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
    public struct AngularConstraint
    {
        // Bodys to be contrained
        Rigid rigid1;
        Rigid rigid2;

        // Inverse inertia tensor of each body
        REAL3x3 rigid1_InertiaInv;
        REAL3x3 rigid2_InertiaInv;

        public AngularConstraint(Rigid Rigid1, Rigid Rigid2)
        {
            this.rigid1 = Rigid1;
            this.rigid2 = Rigid2;

            rigid1_InertiaInv = rigid1.InertiaInv;
            rigid2_InertiaInv = rigid2.InertiaInv;
        }

        public REAL GetDeltaLambda(REAL dt, REAL compliance, REAL lambda, REAL3 delta_q)
        {
            REAL theta = math.length(delta_q);

            if (theta <= Util.EPSILON)
                return 0f;

            REAL3 N = delta_q / theta;
            REAL w1 = math.dot(N, math.mul(rigid1_InertiaInv, N));
            REAL w2 = math.dot(N, math.mul(rigid2_InertiaInv, N));

            if(w1 + w2 < Util.EPSILON)
                return 0f;

            REAL alpha = compliance / (dt * dt);
            REAL d_lambda = (-theta - alpha * lambda) / (w1 + w2 + alpha);
            apply(d_lambda, N);

            return d_lambda;
        }

        private void apply(REAL d_lambda, REAL3 N)
        {
            REAL3 impulse = -d_lambda * N;

            // Update rotation of rigids

            REAL3 I1invP = math.mul(rigid1_InertiaInv, impulse);
            REAL3 I2invP = math.mul(rigid2_InertiaInv, impulse);

            if (!rigid1.isFixed)
            {
                quaternion newQ = new quaternion(0.5f * new float4((float3)I1invP, 0));
                rigid1.Rotation = rigid1.Rotation.value + math.mul(newQ, rigid1.Rotation).value;
                rigid1.Rotation = math.normalizesafe(rigid1.Rotation);
            }

            if (!rigid2.isFixed)
            {
                quaternion newQ = new quaternion(0.5f * new float4((float3)I2invP, 0));
                rigid2.Rotation = rigid2.Rotation.value - math.mul(newQ, rigid2.Rotation).value;
                rigid2.Rotation = math.normalizesafe(rigid2.Rotation);
            }
        }
    }
}

