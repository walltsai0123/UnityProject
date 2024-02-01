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

        public float4 frictionCoef;
        public float restitutionCoef;
        public float3 q;   //Contact point
        public float3 N;   //Surface normal
        private float3 fn;
        public float3 vn_;

        public float3 T;    //Surface tangent
        public float3 B;    //Surface bitangent

        private float tangentCoef = 0f;
        private float bitangentCoef = 0f;

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

            vn_ = math.dot(N, soft.Vel[index]);

            float C = math.dot(soft.Pos[index] - q, N);
            if (C > Util.EPSILON)
                return;

            float alpha = 0.0f;
            float w1 = soft.invMass[index];
            float w2 = 0.0f;
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * N;
            fn = p / (dt * dt);
            soft.Pos[index] += p * w1;

            float3 dp = soft.Pos[index] - soft.prevPos[index];

            // tangent friction
            float dp_t = math.dot(dp, T);
            float C_T = math.abs(dp_t);
            if (C_T > Util.EPSILON)
            {
                float dlambda_t = -dp_t / (w1 + w2 + alpha);
                tangentCoef = (dp_t > 0f) ? frictionCoef[0] : frictionCoef[2];
                //dlambda_t = math.min(dlambda_t, dlambda * tangentCoef);
                dlambda_t = math.max(-frictionCoef[2] * dlambda, math.min(dlambda_t, frictionCoef[0] * dlambda));
                float3 p_t = dlambda_t * T;
                soft.Pos[index] += p_t * w1;
            }

            // bitangent friction
            float dp_b = math.dot(dp, B);
            float C_B = math.abs(dp_b);
            if (C_B > Util.EPSILON)
            {
                float dlambda_b = -dp_b / (w1 + w2 + alpha);
                bitangentCoef = (dp_b > 0f) ? frictionCoef[1] : frictionCoef[3];
                //dlambda_b = math.min(dlambda_b, dlambda * bitangentCoef);
                dlambda_b = math.max(-frictionCoef[3] * dlambda, math.min(dlambda_b, frictionCoef[1] * dlambda));
                float3 p_b = dlambda_b * B;
                soft.Pos[index] += p_b * w1;
            }

        }

        private void SolveSoftBodyVelcity(float dt)
        {
            SoftBody soft = (SoftBody)body;

            float3 v = soft.Vel[index];
            float vn = math.dot(N, v);
            float normalImpulse = dt * math.length(fn);

            //// tangent
            //float vt = math.dot(v, T);
            //float3 dvt = -math.sign(vt) * math.min(normalImpulse * tangentCoef, math.abs(vt)) * T;
            //soft.Vel[index] += dvt;
            //
            //// bitangent
            //float vb = math.dot(v, B);
            //float3 dvb = -math.sign(vb) * math.min(normalImpulse * bitangentCoef, math.abs(vb)) * B;
            //soft.Vel[index] += dvb;

            // normal
            restitutionCoef = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
            float3 dvn = N * (-vn + math.max(0.0f, -restitutionCoef * vn_));
            soft.Vel[index] += dvn;

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
            result += frictionCoef + "\n";
            result += tangentCoef + "\n";
            result += bitangentCoef + "\n";
            result += restitutionCoef + "\n";
            result += q + "\n";
            result += N + "\n";
            result += T + "\n";
            result += B + "\n";
            result += fn + "\n";
            result += vn_ + "\n";
            result += base.ToString();
            return result;
        }
    }
}

