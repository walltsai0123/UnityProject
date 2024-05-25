using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor.Timeline.Actions;
using UnityEngine;

namespace XPBD
{
    public struct AngularConstraint
    {
        // Bodys to be contrained
        Rigid rigid1;
        Rigid rigid2;

        // Inverse inertia tensor of each body
        float3x3 rigid1_InertiaInv;
        float3x3 rigid2_InertiaInv;

        public AngularConstraint(Rigid Rigid1, Rigid Rigid2)
        {
            this.rigid1 = Rigid1;
            this.rigid2 = Rigid2;

            rigid1_InertiaInv = rigid1.InertiaInv;
            rigid2_InertiaInv = rigid2.InertiaInv;
        }

        public float GetDeltaLambda(float dt, float compliance, float lambda, float3 delta_q)
        {
            float theta = math.length(delta_q);

            if (theta <= Util.EPSILON)
                return 0f;

            float3 N = delta_q / theta;
            float w1 = math.dot(N, math.mul(rigid1_InertiaInv, N));
            float w2 = math.dot(N, math.mul(rigid2_InertiaInv, N));

            if(w1 + w2 < Util.EPSILON)
                return 0f;

            float alpha = compliance / (dt * dt);
            float d_lambda = (-theta - alpha * lambda) / (w1 + w2 + alpha);
            apply(d_lambda, N);

            return d_lambda;
        }

        private void apply(float d_lambda, float3 N)
        {
            float3 impulse = -d_lambda * N;

            // Update rotation of rigids

            float3 I1invP = math.mul(rigid1_InertiaInv, impulse);
            float3 I2invP = math.mul(rigid2_InertiaInv, impulse);

            if (!rigid1.isFixed)
            {
                rigid1.Rotation = rigid1.Rotation.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), rigid1.Rotation).value;
                rigid1.Rotation = math.normalizesafe(rigid1.Rotation);
            }

            if (!rigid2.isFixed)
            {
                rigid2.Rotation = rigid2.Rotation.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), rigid2.Rotation).value;
                rigid2.Rotation = math.normalizesafe(rigid2.Rotation);
            }
        }
    }
}

