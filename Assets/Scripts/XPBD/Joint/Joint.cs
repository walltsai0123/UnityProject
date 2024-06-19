using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace XPBD
{
    public abstract class Joint : Constraint
    {
        protected Rigid body1;
        public Rigid body2;

        [SerializeField, Min(0f)]
        protected float linearDamping = 0f;

        [SerializeField, Min(0f)]
        protected float angularDamping = 1f; 

        public override void SolveVelocities(float dt)
        {
            float3 dv = (body2.vel - body1.vel) * math.min(linearDamping * dt, 1f);
            float3 domega = (body2.omega - body1.omega) * math.min(angularDamping * dt, 1f);

            // linear part
            float3 p = dv / (body1.InvMass + body2.InvMass);
            body1.vel += p * body1.InvMass;
            body2.vel -= p * body2.InvMass;

            // angular part
            float3 n = math.normalizesafe(domega, float3.zero);
            float w1 = math.mul(n, math.mul(body1.InertiaInv, n));
            float w2 = math.mul(n, math.mul(body2.InertiaInv, n));

            if (w1 + w2 <= Util.EPSILON)
                return;
            p = domega / (w1 + w2);
            body1.omega += math.mul(body1.InertiaInv, p);
            body2.omega -= math.mul(body2.InertiaInv, p);
        }
        protected void SolvePositionConstraint(float dt, float3 r1, float3 r2, float3 dx, float dmax, float compliance)
        {
            float C = math.length(dx) - dmax;
            if (math.abs(C) < math.EPSILON)
                return;

            quaternion q1 = body1.Rotation;
            quaternion q2 = body2.Rotation;
            float3 R1 = math.rotate(q1, r1);
            float3 R2 = math.rotate(q2, r2);
            float3 n = math.normalize(dx);
            float3x3 I1Inv = math.mul(new float3x3(q1), math.mul(body1.InertiaBodyInv, new float3x3(math.conjugate(q1))));
            float3x3 I2Inv = math.mul(new float3x3(q2), math.mul(body2.InertiaBodyInv, new float3x3(math.conjugate(q2))));

            float3 r1xn = math.cross(R1, n);
            float3 r2xn = math.cross(R2, n);

            float w1 = body1.InvMass + math.mul(r1xn, math.mul(I1Inv, r1xn));
            float w2 = body2.InvMass + math.mul(r2xn, math.mul(I2Inv, r2xn));

            float alpha = compliance / (dt * dt);
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * n;

            body1.Position += p * body1.InvMass;
            body2.Position -= p * body2.InvMass;

            float3 r1xp = math.mul(I1Inv, math.cross(R1, p));
            float3 r2xp = math.mul(I2Inv, math.cross(R2, p));

            quaternion Q1 = q1.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), q1).value;
            quaternion Q2 = q2.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), q2).value;

            body1.Rotation = math.normalizesafe(Q1, quaternion.identity);
            body2.Rotation = math.normalizesafe(Q2, quaternion.identity);
        }
        protected void SolvePositionConstraint3(float dt, float3 r1, float3 r2, float3 dx, float dmax, float compliance)
        {
            // C = ||dx|| - dmax
            float C = math.length(dx) - dmax;
            // if C == 0; return
            if (math.abs(C) < math.EPSILON)
                return;

            quaternion q1 = body1.Rotation;
            quaternion q2 = body2.Rotation;
            // n = dx / ||dx||
            float3 n = math.normalize(dx);
            // n1 = q1^-1 * n
            // n2 = q2^-1 * n
            float3 n1 = math.rotate(math.conjugate(q1), n);
            float3 n2 = math.rotate(math.conjugate(q2), n);

            // Inverse inertia in the local body frame
            float3x3 I1inv = body1.InertiaBodyInv;
            float3x3 I2inv = body2.InertiaBodyInv;

            // r x n
            float3 r1xn = math.cross(r1, n1);
            float3 r2xn = math.cross(r2, n2);

            // w + (r x n)^T * Iinv * (r x n)
            float w1 = body1.InvMass + math.mul(r1xn, math.mul(I1inv, r1xn));
            float w2 = body2.InvMass + math.mul(r2xn, math.mul(I2inv, r2xn));

            float alpha = compliance / (dt * dt);
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * n;

            body1.Position += p * body1.InvMass;
            body2.Position -= p * body2.InvMass;

            // p in local body frame
            float3 p1 = dlambda * n1;
            float3 p2 = dlambda * n2;

            // Iinv * (r x p)
            float3 r1xp = math.mul(I1inv, math.cross(r1, p1));
            float3 r2xp = math.mul(I2inv, math.cross(r2, p2));

            // q_new = q + 0.5 * [Iinv * (r x p), 0] * q
            quaternion Q1 = q1.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), q1).value;
            quaternion Q2 = q2.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), q2).value;

            // normalize q
            body1.Rotation = math.normalizesafe(Q1, quaternion.identity);
            body2.Rotation = math.normalizesafe(Q2, quaternion.identity);

        }

        protected void SolveAngularConstraint(float dt, float3 dq, float angle, float compliance)
        {
            float C = math.length(dq) - angle;
            if (C == 0f)
                return;

            quaternion q1 = body1.Rotation;
            quaternion q2 = body2.Rotation;

            float3 n = math.normalize(dq);

            float3x3 I1Inv = math.mul(new float3x3(q1), math.mul(body1.InertiaBodyInv, new float3x3(math.conjugate(q1))));
            float3x3 I2Inv = math.mul(new float3x3(q2), math.mul(body2.InertiaBodyInv, new float3x3(math.conjugate(q2))));

            float w1 = math.mul(n, math.mul(I1Inv, n));
            float w2 = math.mul(n, math.mul(I2Inv, n));

            float alpha = compliance / (dt * dt);
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * n;

            float3 I1invP = math.mul(I1Inv, p);
            float3 I2invP = math.mul(I2Inv, p);

            quaternion Q1 = q1.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), q1).value;
            quaternion Q2 = q2.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), q2).value;

            body1.Rotation = math.normalizesafe(Q1, quaternion.identity);
            body2.Rotation = math.normalizesafe(Q2, quaternion.identity);
        }

        protected void SolveAngularConstraint3(float dt, float3 dq, float angle, float compliance)
        {
            float C = math.length(dq) - angle;
            if (C == 0f)
                return;

            quaternion q1 = body1.Rotation;
            quaternion q2 = body2.Rotation;

            float3 n = math.normalize(dq);
            // n1 = q1^-1 * n
            // n2 = q2^-1 * n
            float3 n1 = math.rotate(math.conjugate(q1), n);
            float3 n2 = math.rotate(math.conjugate(q2), n);

            float3x3 I1Inv = body1.InertiaBodyInv;
            float3x3 I2Inv = body2.InertiaBodyInv;

            float w1 = math.mul(n1, math.mul(I1Inv, n1));
            float w2 = math.mul(n2, math.mul(I2Inv, n2));

            float alpha = compliance / (dt * dt);
            float dlambda = -C / (w1 + w2 + alpha);
            float3 p = dlambda * n;

            float3 p1 = math.rotate(math.conjugate(q1), p);
            float3 p2 = math.rotate(math.conjugate(q2), p);

            float3 I1invP = math.mul(I1Inv, p1);
            float3 I2invP = math.mul(I2Inv, p2);

            quaternion Q1 = q1.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), q1).value;
            quaternion Q2 = q2.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), q2).value;

            body1.Rotation = math.normalizesafe(Q1, quaternion.identity);
            body2.Rotation = math.normalizesafe(Q2, quaternion.identity);
        }
    }
}

