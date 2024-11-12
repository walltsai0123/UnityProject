using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using UnityEditor.MPE;
using Unity.Burst;





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
    public struct AngularConstraint
    {
        // Bodys to be contrained
        //Rigid rigid1;
        //Rigid rigid2;

        // Inverse inertia tensor of each body
        public REAL3x3 rigid1_InertiaInv;
        public REAL3x3 rigid2_InertiaInv;

        public quaternion q1, q2;

        public AngularConstraint(Rigid Rigid1, Rigid Rigid2)
        {
            //this.rigid1 = Rigid1;
            //this.rigid2 = Rigid2;

            q1 = Rigid1.Rotation;
            q2 = Rigid2.Rotation;

            rigid1_InertiaInv = Rigid1.InertiaInv;
            rigid2_InertiaInv = Rigid2.InertiaInv;
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

            quaternion newQ = new quaternion(0.5f * new float4((float3)I1invP, 0));
            q1 = q1.value + math.mul(newQ, q1).value;
            q1 = math.normalizesafe(q1);

            quaternion newQ2 = new quaternion(0.5f * new float4((float3)I2invP, 0));
            q2 = q2.value - math.mul(newQ2, q2).value;
            q2 = math.normalizesafe(q2);
        }
    }
}

