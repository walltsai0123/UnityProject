using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class MyCollision : CollisionConstraint
    {
        private Body body;
        private Primitive primitive;
        private int index;

        public float frictionCoef;
        public float restitutionCoef;
        public float3 q;   //Contact point
        public float3 N;   //Surface normal
        private float3 fn;
        public float3 vn_;

        public MyCollision(Body b, Primitive p, int i = -1)
        {
            body = b;
            primitive = p;
            index = i;
            q = float3.zero;
            N = float3.zero;
            frictionCoef = 0.4f;
            fn = float3.zero;
        }

        public override void SolveCollision(float dt)
        {
            if (body.bodyType == Body.BodyType.Soft)
                SolveSoftBodyCollision(dt);
        }

        public override void VelocitySolve(float dt)
        {
            if (body.bodyType == Body.BodyType.Soft)
                SolveSoftBodyVelcity(dt);
        }

        private void SolveSoftBodyCollision(float dt)
        {
            SoftBody soft = (SoftBody)body;

            //if (!Util.IsInsideCollider(primitive.collider, soft.Pos[index]))
            //    return;

            //float3 direction = soft.Pos[index] - soft.prevPos[index];
            //Ray ray = new Ray(soft.prevPos[index], direction);
            //RaycastHit hit;

            //if (primitive.collider.Raycast(ray, out hit, math.length(direction)))
            //{
            //    q = hit.point;
            //    N = hit.normal;
            //}
            //else    // Ray inside or too short
            //{
            //    q = primitive.collider.ClosestPoint(soft.Pos[index]);
            //    N = math.normalizesafe(q - soft.Pos[index], float3.zero);
            //}
            vn_ = math.dot(N, soft.vel[index]);

            float C = math.dot(soft.Pos[index] - q, N);
            if (C >= 0.0f)
                return;

            float alpha = 0.0f;
            float w1 = soft.invMass[index];
            float w2 = 0.0f;
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * N;
            fn = p / (dt * dt);
            soft.Pos[index] += p * w1;

            float3 dp = soft.Pos[index] - soft.prevPos[index];
            float3 dp_t = dp - math.dot(dp, N) * N;

            float C2 = math.length(dp_t);
            if (C2 <= 1e-6f)
                return;

            float dlambda_t = -C2 / (w1 + w2 + alpha);
            dlambda_t = math.min(dlambda_t, dlambda * frictionCoef);
            float3 p2 = dlambda_t * math.normalizesafe(dp_t, float3.zero);
            soft.Pos[index] += p2 * w1;
        }

        private void SolveSoftBodyVelcity(float dt)
        {
            SoftBody soft = (SoftBody)body;

            float3 v = soft.vel[index];
            float vn = math.dot(N, v);
            float3 vt = v - vn * N;

            // tangent
            float3 dvt = -math.normalizesafe(vt, float3.zero) * math.min(dt * frictionCoef * math.length(fn), math.length(vt));
            soft.vel[index] += dvt;

            // normal
            restitutionCoef = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
            float3 dvn = N * (-vn + math.max(0.0f, -restitutionCoef * vn_));
            soft.vel[index] += dvn;

        }

        public void PrintData()
        {
            string data = "";
            data += ToString() + "\n";
            data += frictionCoef + "\n";
            data += restitutionCoef + "\n";
            data += q + "\n";
            data += N + "\n";
            data += fn + "\n";
            data += vn_ + "\n";

            Debug.Log(data);
        }
        public override string ToString()
        {
            string result = body.ToString() + "\n";
            result += primitive.ToString() + "\n";
            result += index + "\n";
            result += base.ToString();
            return result;
        }
    }
}

