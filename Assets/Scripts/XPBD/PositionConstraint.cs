using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public struct PositionConstraint
    {
        // Bodys to be contrained
        Rigid rigid1;
        Rigid rigid2;

        // Constraint point on each body in world coordinate
        public float3 r1;
        public float3 r2;

        // Inverse inertia tensor of each body
        float3x3 rigid1_InertiaInv;
        float3x3 rigid2_InertiaInv;

        public PositionConstraint(Rigid Rigid1, Rigid Rigid2, float3 r1_lc, float3 r2_lc)
        {
            this.rigid1 = Rigid1;
            this.rigid2 = Rigid2;

            r1 = math.rotate(rigid1.Rotation, r1_lc);
            r2 = math.rotate(rigid2.Rotation, r2_lc);

            rigid1_InertiaInv = rigid1.InertiaInv;
            rigid2_InertiaInv = rigid2.InertiaInv;
        }

        public float GetDeltaLambda(float dt, float compliance, float lambda, float3 delta_x)
        {
            float C = math.length(delta_x);

            if(C <= Util.EPSILON)
            {
                return 0f;
            }

            float3 N = delta_x / C;

            float w1 = rigid1.InvMass + math.dot(math.cross(r1, N), math.mul(rigid1_InertiaInv, math.cross(r1, N)));
            float w2 = rigid2.InvMass + math.dot(math.cross(r2, N), math.mul(rigid2_InertiaInv, math.cross(r2, N)));

            if(w1 + w2 < Util.EPSILON)
            {
                return 0f;
            }

            float alpha = compliance / (dt * dt);
            float d_lambda = (-C - alpha * lambda) / (w1 + w2 + alpha);

            // Apply change to rigids
            apply(d_lambda, N);

            return d_lambda;
        }

        private void apply(float d_lambda, float3 N)
        {
            float3 impulse = d_lambda * N;

            // Update position of rigids
            if(!rigid1.isFixed)
                rigid1.Position += rigid1.InvMass * impulse;

            if(!rigid2.isFixed)
                rigid2.Position += -rigid2.InvMass * impulse;

            // Update rotation of rigids

            float3 r1xp = math.mul(rigid1_InertiaInv, math.cross(r1, impulse));
            float3 r2xp = math.mul(rigid2_InertiaInv, math.cross(r2, impulse));

            if (!rigid1.isFixed)
            {
                rigid1.Rotation = rigid1.Rotation.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), rigid1.Rotation).value;
                rigid1.Rotation = math.normalizesafe(rigid1.Rotation);
            }

            if (!rigid2.isFixed)
            {
                rigid2.Rotation = rigid2.Rotation.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), rigid2.Rotation).value;
                rigid2.Rotation = math.normalizesafe(rigid2.Rotation);
            }
        }
    }
}

