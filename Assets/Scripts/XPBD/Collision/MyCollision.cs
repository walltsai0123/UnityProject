using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using Unity.VisualScripting;
using UnityEditor.ShaderGraph.Internal;




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
    public class MyCollision
    {
        public SoftBody softBody;
        public MyTerrain terrain;
        public int index;
        public REAL4 frictionCoef;
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

        public REAL lambda_n = 0f;
        public REAL lambda_t = 0f;
        public REAL lambda_b = 0f;

        public REAL F_lambda = 0f;
        public REAL3 F = 0f;

        public REAL shearDisplacement = 0;

        public MyCollision(int i = -1)
        {
            index = i;
            q = REAL3.zero;
            N = REAL3.zero;
            frictionCoef = 0.5;
            fn = ft = fb = REAL3.zero;
        }

        public REAL3 ConstraintForce(REAL dt)
        {
            return (N * lambda_n + T * lambda_t + B * lambda_b) / dt / dt;
        }
        public REAL3 ConstraintNormalForce(REAL dt)
        {
            return N * lambda_n / dt / dt;
        }
        public REAL3 ConstraintFrictionForce(REAL dt)
        {
            return (T * lambda_t + B * lambda_b) / dt / dt;
        }
        public void SolveCollision(SoftBodySystem sbs, REAL dt)
        {
            SoftBodyParticle particle = sbs.particles[index];

            vn_ = math.dot(N, particle.vel);

            REAL C = math.dot(particle.pos - q, N);
            if (C > Util.EPSILON)
                return;

            REAL h2 = dt * dt;
            REAL alpha = 0.0f;
            REAL w1 = particle.invMass;
            REAL w2 = 0.0f;
            REAL dlambda = -C / (w1 + w2 + alpha);

            REAL pressure_dlambda = PressureSinkageConstraintSolve(particle, dt, out float pressure_lambda);

            REAL final_dlambda = (terrain.GroundMaterial.n >= 1) ? math.max(dlambda, pressure_dlambda) : math.min(dlambda, pressure_dlambda);
            final_dlambda = dlambda;

            REAL3 p = final_dlambda * N;
            fn = p / h2;
            //fn = pressure_lambda / h2 * N;
            lambda_n += final_dlambda;
            particle.pos += p * w1;

            REAL3 dp = particle.pos - q;

            REAL pressure = math.length(fn) / terrain.CellArea;
            REAL maxShearForce = terrain.GetMaxShear((float)pressure);
            frictionCoef = maxShearForce / pressure;


            // tangent friction
            REAL dp_t = math.dot(dp, T);
            REAL C_T = math.abs(dp_t);
            ft = REAL3.zero;
            if (C_T > Util.EPSILON)
            {
                REAL dlambda_t = -dp_t / (w1 + w2 + alpha);
                tangentCoef = (dp_t > 0f) ? frictionCoef[0] : frictionCoef[2];
                dlambda_t = math.max(-frictionCoef[2] * dlambda, math.min(dlambda_t, frictionCoef[0] * dlambda));
                REAL3 p_t = dlambda_t * T;
                particle.pos += p_t * w1;
                ft = p_t / h2;
                lambda_t += dlambda_t;
            }

            // bitangent friction
            REAL dp_b = math.dot(dp, B);
            REAL C_B = math.abs(dp_b);
            fb = REAL3.zero;
            if (C_B > Util.EPSILON)
            {
                REAL dlambda_b = -dp_b / (w1 + w2 + alpha);
                bitangentCoef = (dp_b > 0f) ? frictionCoef[1] : frictionCoef[3];
                dlambda_b = math.max(-frictionCoef[3] * dlambda, math.min(dlambda_b, frictionCoef[1] * dlambda));
                REAL3 p_b = dlambda_b * B;
                particle.pos += p_b * w1;
                fb = p_b / h2;
                lambda_b += dlambda_b;
            }
            F += (fn + ft + fb);

            dp = particle.pos - q;
            shearDisplacement += math.length(dp) - (float)math.dot(dp, N);
            sbs.particles[index] = particle;
        }

        public void SolveCollision1(SoftBodySystem sbs, REAL dt)
        {
            SoftBodyParticle particle = sbs.particles[index];

            vn_ = math.dot(N, particle.vel);

            REAL penetration = math.dot(particle.pos - q, N);
            if (penetration > Util.EPSILON)
                return;

            REAL C = -math.pow(math.abs(penetration), (terrain.GroundMaterial.n + 1) / 2);
            REAL gradC = math.pow(math.abs(penetration), (terrain.GroundMaterial.n - 1) / 2) * (terrain.GroundMaterial.n + 1) / 2;

            REAL cos_theta = math.abs(math.dot(new(0, 1, 0), N));
            REAL contactSurface = terrain.CellArea;
            REAL stiffness = 2 * contactSurface * terrain.Stiffness / (terrain.GroundMaterial.n + 1);
            REAL compliance = (stiffness == 0) ? 0 : 1 / stiffness;

            REAL h2 = dt * dt;
            REAL alpha = compliance / h2;
            REAL w1 = particle.invMass * gradC * gradC;
            REAL dlambda = -C / (w1 + alpha);
            REAL3 p = dlambda * gradC * N;
            fn = p / h2;
            particle.pos += p * particle.invMass;

            lambda_n += dlambda;
            F_lambda += dlambda / h2;

            REAL3 dp = particle.pos - q;
            REAL pressure = math.length(fn) / contactSurface;
            REAL maxShearForce = terrain.GetMaxShear((float)pressure) * contactSurface * h2;

            //friction
            REAL3 displacement = dp - math.dot(dp, N) * N;
            REAL j = math.length(displacement);
            REAL C_T = j;
            ft = REAL3.zero;
            if (j > Util.EPSILON)
            {
                REAL3 dir = math.normalize(displacement);

                REAL w_T = particle.invMass;
                REAL compliance_T = 0;

                REAL alpha_T = compliance_T / h2;
                REAL dlambda_t = -C_T / (w_T + alpha_T);

                REAL shearForceLimit = maxShearForce;
                dlambda_t = math.clamp(dlambda_t, -shearForceLimit, shearForceLimit);
                REAL3 p_t = dlambda_t * dir;


                particle.pos += p_t * particle.invMass;

                ft = p_t / h2;
                tangentCoef = bitangentCoef = math.abs(math.length(ft) / math.length(fn));
            }
            F = 0;
            F += fn + ft + fb;
            dp = particle.pos - q;
            shearDisplacement += math.length(dp - (float)math.dot(dp, N) * N);
            sbs.particles[index] = particle;
        }

        REAL PressureSinkageConstraintSolve(SoftBodyParticle particle, REAL dt, out float lambda)
        {
            lambda = 0;
            REAL penetration = math.dot(particle.pos - q, N);
            if (penetration > Util.EPSILON)
                return 0;

            REAL C = -math.pow(math.abs(penetration), (terrain.GroundMaterial.n + 1) / 2);
            REAL gradC = math.pow(math.abs(penetration), (terrain.GroundMaterial.n - 1) / 2) * (terrain.GroundMaterial.n + 1) / 2;

            REAL contactSurface = terrain.CellArea;
            REAL stiffness = 2 * contactSurface * terrain.Stiffness / (terrain.GroundMaterial.n + 1);
            REAL compliance = (stiffness == 0) ? 0 : 1 / stiffness;

            REAL h2 = dt * dt;
            REAL alpha = compliance / h2;
            REAL w1 = particle.invMass * gradC * gradC;
            REAL dlambda = -C / (w1 + alpha);
            REAL p = dlambda * gradC;
            lambda = (float)dlambda;
            return p;
        }
        public void VelocitySolve(SoftBodySystem sbs, REAL dt) 
        {
            SoftBodyParticle particle = sbs.particles[index];

            REAL3 v = particle.vel;
            REAL vn = math.dot(N, v);
            REAL normalImpulse = dt * math.length(fn);

            if (normalImpulse < Util.EPSILON)
                return;

            //REAL3 vt = v - vn;
            //REAL3 dir_t = math.normalizesafe(vt, 0);
            //REAL vt_length = math.length(vt);
            //REAL3 dvt = -dir_t * math.min(normalImpulse * tangentCoef, vt_length);
            //particle.vel += dvt;
            // tangent
            REAL vt = math.dot(v, T);
            REAL3 dvt = -math.sign(vt) * math.min(normalImpulse * tangentCoef, math.abs(vt)) * T;
            particle.vel += dvt;
            
            // bitangent
            REAL vb = math.dot(v, B);
            REAL3 dvb = -math.sign(vb) * math.min(normalImpulse * bitangentCoef, math.abs(vb)) * B;
            particle.vel += dvb;

            // normal
            restitutionCoef = (math.abs(vn) <= 2.0f * 9.81f * dt) ? 0.0f : 1.0f;
            REAL3 dvn = N * (-vn + math.max(0.0f, -restitutionCoef * vn_));
            particle.vel += dvn;

            sbs.particles[index] = particle;
        }

        public override string ToString()
        {
            string result = "";
            //result += primitive.ToString() + "\n";
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

