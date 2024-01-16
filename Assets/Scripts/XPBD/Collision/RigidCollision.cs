using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class RigidCollision : CollisionConstraint
    {
        Rigid rigid;
        Primitive primitive;
        float3 point;
        float3 normal;
        float penetration;
        float3 r1;
        float fn = 0f;

        float frictionCoef = 0.4f;
        float restitutionCoef = 0.25f;
        float vn_;

        public RigidCollision(Rigid rigid, Primitive primitive, ContactPoint contactPoint)
        {
            this.rigid = rigid;
            this.primitive = primitive;
            point = contactPoint.point;
            normal = contactPoint.normal;
            penetration = contactPoint.separation;
            float3 R1 = (point + penetration * normal) - rigid.Position;
            r1 = math.rotate(math.conjugate(rigid.Rotation), R1);
        }

        public override void SolveCollision(float dt)
        {
            float3 p1 = rigid.Position + math.rotate(rigid.Rotation, r1);
            float C = math.dot(p1 - point, normal);
            if (C >= 0)
                return;


            float3 n = math.normalize(normal);
            float3 R1 = math.rotate(rigid.Rotation, r1);

            float3 v = rigid.vel + math.cross(rigid.omega, R1);
            vn_ = math.dot(normal, v);


            float3x3 I1Inv = math.mul(new float3x3(rigid.Rotation), 
                math.mul(rigid.InertiaBodyInv, new float3x3(math.conjugate(rigid.Rotation))));

            float3 r1xn = math.cross(R1, n);

            float w1 = rigid.InvMass + math.mul(r1xn, math.mul(I1Inv, r1xn));

            float alpha = 0f;
            float dlambda = -C / (w1 + alpha);
            float3 p = dlambda * n;
            fn = dlambda / (dt * dt);

            rigid.Position += p * rigid.InvMass;

            float3 r1xp = math.mul(I1Inv, math.cross(R1, p));
            quaternion Q1 = rigid.Rotation.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), rigid.Rotation).value;

            rigid.Rotation = math.normalizesafe(Q1, quaternion.identity);

            p1 = rigid.Position + math.rotate(rigid.Rotation, r1);
            float3 p1_bar = rigid.prevPos + math.rotate(rigid.prevRot, r1);

            float3 dp = p1 - p1_bar;
            float3 dp_t = dp - math.dot(dp, n) * n;

            float C2 = math.length(dp_t);

            if (C2 <= 1e-6f)
                return;

            float dlambda_t = -C2 / (w1 + alpha);
            dlambda_t = math.min(dlambda_t, dlambda * frictionCoef);
            float3 P2 = dlambda_t * math.normalizesafe(dp_t, float3.zero);

            rigid.Position += P2 * rigid.InvMass;

            r1xp = math.mul(I1Inv, math.cross(R1, P2));
            Q1 = rigid.Rotation.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), rigid.Rotation).value;

            rigid.Rotation = math.normalizesafe(Q1, quaternion.identity);
        }

        public override void VelocitySolve(float dt)
        {
            float3 R1 = math.rotate(rigid.Rotation, r1);
            float3 v = rigid.vel + math.cross(rigid.omega, R1);
            float vn = math.dot(normal, v);
            float3 vt = v - vn * normal;

            float3x3 I1Inv = math.mul(new float3x3(rigid.Rotation),
                math.mul(rigid.InertiaBodyInv, new float3x3(math.conjugate(rigid.Rotation))));

            float3 r1xn = math.cross(R1, normal);

            float w1 = rigid.InvMass + math.mul(r1xn, math.mul(I1Inv, r1xn));

            // tangent
            float3 dvt = -math.normalizesafe(vt, float3.zero) * math.min(dt * frictionCoef * math.length(fn), math.length(vt));
            float3 p = dvt / w1;
            rigid.vel += p * rigid.InvMass;
            rigid.omega += math.mul(I1Inv, math.cross(R1, p));
            

            // normal
            float e = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : restitutionCoef;
            float3 dvn = normal * (-vn + math.max(0.0f, -e * vn_));
            p = dvn / w1;
            rigid.vel += p * rigid.InvMass;
            rigid.omega += math.mul(I1Inv, math.cross(R1, p));

            //Debug.Log("restitutionCoef: " + e);
            //Debug.Log("vn: " + vn);
            //Debug.Log("vn_: " + vn_);
            //Debug.Log("dvn: " + dvn);
            //Debug.Log("vel: " + rigid.vel);
        }

    }
}

