using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;

namespace XPBD
{
    public abstract class Constraint : MonoBehaviour
    {
        public virtual void SolveConstraint(float dt)
        {
            Debug.LogWarning("Base Constraint SolveConstraint");
        }

        protected struct PositionConstraintData
        {
            public float3 x1, x2;
            public float3 r1, r2;
            public quaternion q1, q2;
            public float invMass1, invMass2;
            public float3x3 IBodyInv1, IBodyInv2;

            public PositionConstraintData(Rigid b1, Rigid b2, float3 R1, float3 R2)
            {
                x1 = b1.Position;
                x2 = b2.Position;
                r1 = R1;
                r2 = R2;
                q1 = b1.Rotation;
                q2 = b2.Rotation;
                invMass1 = b1.InvMass;
                invMass2 = b2.InvMass;
                IBodyInv1 = b1.InertiaBodyInv;
                IBodyInv2 = b2.InertiaBodyInv;
            }
        }
        [BurstCompile]
        protected struct PositionConstraintJob : IJob
        {
            public float dmax;
            public float compliance;
            public float dt;
            public NativeArray<PositionConstraintData> Datas;
            public void Execute()
            {
                PositionConstraintData data = Datas[0];
                float3 R1 = math.rotate(Datas[0].q1, Datas[0].r1);
                float3 R2 = math.rotate(Datas[0].q2, Datas[0].r2);

                float3 dx = (Datas[0].x1 + R1) - (Datas[0].x2 + R2);
                float C = math.length(dx) - dmax;

                if (math.abs(C) < math.EPSILON)
                    return;

                float3 n = math.normalize(dx);
                float3x3 I1Inv = math.mul(new float3x3(Datas[0].q1), math.mul(Datas[0].IBodyInv1, new float3x3(math.conjugate(Datas[0].q1))));
                float3x3 I2Inv = math.mul(new float3x3(Datas[0].q2), math.mul(Datas[0].IBodyInv2, new float3x3(math.conjugate(data.q2))));

                float3 r1xn = math.cross(R1, n);
                float3 r2xn = math.cross(R2, n);

                float w1 = data.invMass1 + math.mul(r1xn, math.mul(I1Inv, r1xn));
                float w2 = data.invMass2 + math.mul(r2xn, math.mul(I2Inv, r2xn));

                float alpha = compliance / (dt * dt);
                float dlambda = (-C - alpha) / (w1 + w2 + alpha);
                float3 p = dlambda * n;

                data.x1 += p * data.invMass1;
                data.x2 -= p * data.invMass2;

                float3 r1xp = math.mul(I1Inv, math.cross(R1, p));
                float3 r2xp = math.mul(I2Inv, math.cross(R2, p));

                quaternion Q1 = data.q1.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), data.q1).value;
                quaternion Q2 = data.q2.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), data.q2).value;

                data.q1 = math.normalizesafe(Q1, quaternion.identity);
                data.q2 = math.normalizesafe(Q2, quaternion.identity);

                Datas[0] = data;
            }
        }

        [BurstCompile]
        protected struct AngularConstraintJob : IJob
        {
            public quaternion q1, q2;
            public float3x3 IBodyInv1, IBodyInv2;
            public float3 dq;
            public float angle;

            public float compliance;
            public float dt;

            public void Execute()
            {
                float C = math.length(dq) - angle;
                if (math.abs(C) < math.EPSILON)
                    return;

                float3 n = math.normalize(dq);

                float3x3 I1Inv = math.mul(new float3x3(q1), math.mul(IBodyInv1, new float3x3(math.conjugate(q1))));
                float3x3 I2Inv = math.mul(new float3x3(q2), math.mul(IBodyInv2, new float3x3(math.conjugate(q2))));

                float w1 = math.mul(n, math.mul(I1Inv, n));
                float w2 = math.mul(n, math.mul(I2Inv, n));

                float alpha = compliance / (dt * dt);
                float dlambda = (-C - alpha) / (w1 + w2 + alpha);
                float3 p = dlambda * n;

                float3 I1invP = math.mul(I1Inv, p);
                float3 I2invP = math.mul(I2Inv, p);

                quaternion Q1 = q1.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), q1).value;
                quaternion Q2 = q2.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), q2).value;

                q1 = math.normalizesafe(Q1, quaternion.identity);
                q2 = math.normalizesafe(Q2, quaternion.identity);
            }
        }
    }
}

