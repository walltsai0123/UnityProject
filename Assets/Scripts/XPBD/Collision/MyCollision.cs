using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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
    public class MyCollision : CollisionConstraint
    {
        public SoftBody body;
        public Primitive primitive;
        public int index;

        //public REAL4 frictionCoef;
        public REAL restitutionCoef;
        public REAL3 q;   //Contact point
        public REAL3 fn;
        public REAL3 ft, fb;
        public REAL3 vn_;

        public REAL3 N;    //Surface normal
        public REAL3 T;    //Surface tangent
        public REAL3 B;    //Surface bitangent

        private REAL tangentCoef = 0f;
        private REAL bitangentCoef = 0f;

        public MyCollision(SoftBody b, Primitive p, int i = -1)
        {
            body = b;
            primitive = p;
            index = i;
            q = REAL3.zero;
            N = REAL3.zero;
            frictionCoef = 0.5f;
            fn = ft = fb = REAL3.zero;
        }

        public override void SolveCollision(REAL dt)
        {
            if (body.bodyType == Body.BodyType.Soft)
                SolveSoftBodyCollision(dt);
        }

        public override void VelocitySolve(REAL dt)
        {
            if (body.bodyType == Body.BodyType.Soft)
                SolveSoftBodyVelcity(dt);
        }

        private void SolveSoftBodyCollision(REAL dt)
        {
            SoftBody soft = (SoftBody)body;

            vn_ = math.dot(N, soft.Vel[index]);

            REAL C = math.dot(soft.Pos[index] - q, N);
            if (C > Util.EPSILON)
                return;

            REAL h2 = dt * dt;
            REAL alpha = 0.0f;
            REAL w1 = soft.invMass[index];
            REAL w2 = 0.0f;
            REAL dlambda = -C / (w1 + w2 + alpha);
            REAL3 p = dlambda * N;
            fn = p / h2;
            soft.Pos[index] += p * w1;

            REAL3 dp = soft.Pos[index] - soft.prevPos[index];

            // tangent friction
            REAL dp_t = math.dot(dp, T);
            REAL C_T = math.abs(dp_t);
            ft = REAL3.zero;
            if (C_T > Util.EPSILON)
            {
                REAL dlambda_t = -dp_t / (w1 + w2 + alpha);
                tangentCoef = (dp_t > 0f) ? frictionCoef[0] : frictionCoef[2];
                //dlambda_t = math.min(dlambda_t, dlambda * tangentCoef);
                dlambda_t = math.max(-frictionCoef[2] * dlambda, math.min(dlambda_t, frictionCoef[0] * dlambda));
                REAL3 p_t = dlambda_t * T;
                soft.Pos[index] += p_t * w1;
                ft = p_t / h2;
            }

            // bitangent friction
            REAL dp_b = math.dot(dp, B);
            REAL C_B = math.abs(dp_b);
            fb = REAL3.zero;
            if (C_B > Util.EPSILON)
            {
                REAL dlambda_b = -dp_b / (w1 + w2 + alpha);
                bitangentCoef = (dp_b > 0f) ? frictionCoef[1] : frictionCoef[3];
                //dlambda_b = math.min(dlambda_b, dlambda * bitangentCoef);
                dlambda_b = math.max(-frictionCoef[3] * dlambda, math.min(dlambda_b, frictionCoef[1] * dlambda));
                REAL3 p_b = dlambda_b * B;
                soft.Pos[index] += p_b * w1;
                fb = p_b / h2;
            }

        }

        private void SolveSoftBodyVelcity(REAL dt)
        {
            SoftBody soft = (SoftBody)body;

            REAL3 v = soft.Vel[index];
            REAL vn = math.dot(N, v);
            REAL normalImpulse = dt * math.length(fn);

            if (normalImpulse < Util.EPSILON)
                return;

            // tangent
            REAL vt = math.dot(v, T);
            REAL3 dvt = -math.sign(vt) * math.min(normalImpulse * tangentCoef, math.abs(vt)) * T;
            soft.Vel[index] += dvt;
            
            // bitangent
            REAL vb = math.dot(v, B);
            REAL3 dvb = -math.sign(vb) * math.min(normalImpulse * bitangentCoef, math.abs(vb)) * B;
            soft.Vel[index] += dvb;

            // normal
            restitutionCoef = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
            REAL3 dvn = N * (-vn + math.max(0.0f, -restitutionCoef * vn_));
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

