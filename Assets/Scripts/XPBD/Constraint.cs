using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;


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
    public abstract class Constraint : MonoBehaviour
    {
        //[SerializeField] protected float compliance = 0f;
        protected float lambda = 0f;

        protected NativeArray<PositionConstraint> positionConstraints;
        protected NativeArray<AngularConstraint> angularConstraints;

        void OnEnable()
        {
            Initialize();
            Simulation.get.AddConstraints(this);

            positionConstraints = new(1, Allocator.Persistent);
            angularConstraints = new(1, Allocator.Persistent);
        }
        private void OnDisable()
        {
            positionConstraints.Dispose();
            angularConstraints.Dispose();
        }

        public virtual void ResetLambda()
        {
            lambda = 0f;
        }
        public virtual void SolveConstraint(REAL dt)
        {
            Debug.LogWarning("Base Constraint SolveConstraint");
        }

        public abstract void SolveVelocities(REAL dt);

        protected abstract void Initialize();

        //protected struct PositionConstraintData
        //{
        //    public float3 x1, x2;
        //    public float3 r1, r2;
        //    public quaternion q1, q2;
        //    public float invMass1, invMass2;
        //    public float3x3 IBodyInv1, IBodyInv2;

        //    public PositionConstraintData(Rigid b1, Rigid b2, float3 R1, float3 R2)
        //    {
        //        x1 = b1.Position;
        //        x2 = b2.Position;
        //        r1 = R1;
        //        r2 = R2;
        //        q1 = b1.Rotation;
        //        q2 = b2.Rotation;
        //        invMass1 = b1.InvMass;
        //        invMass2 = b2.InvMass;
        //        IBodyInv1 = b1.InertiaBodyInv;
        //        IBodyInv2 = b2.InertiaBodyInv;
        //    }
        //}

        //[BurstCompile]
        //protected struct PositionConstraintJob : IJob
        //{
        //    public float3 dx;
        //    public float dmax;
        //    public float compliance;
        //    public float dt;
        //    public NativeArray<PositionConstraintData> Datas;
        //    public void Execute()
        //    {
        //        float C = math.length(dx) - dmax;
        //        if (math.abs(C) < math.EPSILON)
        //            return;

        //        PositionConstraintData data = Datas[0];
        //        float3 R1 = math.rotate(data.q1, data.r1);
        //        float3 R2 = math.rotate(data.q2, data.r2);
        //        float3 n = math.normalize(dx);
        //        float3x3 I1Inv = math.mul(new float3x3(data.q1), math.mul(data.IBodyInv1, new float3x3(math.conjugate(data.q1))));
        //        float3x3 I2Inv = math.mul(new float3x3(data.q2), math.mul(data.IBodyInv2, new float3x3(math.conjugate(data.q2))));

        //        float3 r1xn = math.cross(R1, n);
        //        float3 r2xn = math.cross(R2, n);

        //        float w1 = data.invMass1 + math.mul(r1xn, math.mul(I1Inv, r1xn));
        //        float w2 = data.invMass2 + math.mul(r2xn, math.mul(I2Inv, r2xn));

        //        float alpha = compliance / (dt * dt);
        //        float dlambda = -C / (w1 + w2 + alpha);
        //        float3 p = dlambda * n;

        //        data.x1 += p * data.invMass1;
        //        data.x2 -= p * data.invMass2;

        //        float3 r1xp = math.mul(I1Inv, math.cross(R1, p));
        //        float3 r2xp = math.mul(I2Inv, math.cross(R2, p));

        //        quaternion Q1 = data.q1.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), data.q1).value;
        //        quaternion Q2 = data.q2.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), data.q2).value;

        //        data.q1 = math.normalizesafe(Q1, quaternion.identity);
        //        data.q2 = math.normalizesafe(Q2, quaternion.identity);

        //        Datas[0] = data;
        //    }

        //    public void Execute3()
        //    {
        //        // C = ||dx|| - dmax
        //        float C = math.length(dx) - dmax;
        //        // if C == 0; return
        //        if (math.abs(C) < math.EPSILON)
        //            return;

        //        PositionConstraintData data = Datas[0];
        //        // n = dx / ||dx||
        //        float3 n = math.normalize(dx);
        //        // n1 = q1^-1 * n
        //        // n2 = q2^-1 * n
        //        float3 n1 = math.rotate(math.conjugate(data.q1), n);
        //        float3 n2 = math.rotate(math.conjugate(data.q2), n);

        //        // Inverse inertia in the local body frame
        //        float3x3 I1inv = data.IBodyInv1;
        //        float3x3 I2inv = data.IBodyInv2;

        //        // r x n
        //        float3 r1xn = math.cross(data.r1, n1);
        //        float3 r2xn = math.cross(data.r2, n2);

        //        // w + (r x n)^T * Iinv * (r x n)
        //        float w1 = data.invMass1 + math.mul(r1xn, math.mul(I1inv, r1xn));
        //        float w2 = data.invMass2 + math.mul(r2xn, math.mul(I2inv, r2xn));

        //        float alpha = compliance / (dt * dt);
        //        float dlambda = -C / (w1 + w2 + alpha);
        //        float3 p = dlambda * n;

        //        data.x1 += p * data.invMass1;
        //        data.x2 -= p * data.invMass2;

        //        // p in local body frame
        //        float3 p1 = dlambda * n1;
        //        float3 p2 = dlambda * n2;

        //        // Iinv * (r x p)
        //        float3 r1xp = math.mul(I1inv, math.cross(data.r1, p1));
        //        float3 r2xp = math.mul(I2inv, math.cross(data.r2, p2));

        //        // q_new = q + 0.5 * [Iinv * (r x p), 0] * q
        //        quaternion Q1 = data.q1.value + math.mul(new quaternion(0.5f * new float4(r1xp, 0f)), data.q1).value;
        //        quaternion Q2 = data.q2.value - math.mul(new quaternion(0.5f * new float4(r2xp, 0f)), data.q2).value;

        //        // normalize q
        //        data.q1 = math.normalizesafe(Q1, quaternion.identity);
        //        data.q2 = math.normalizesafe(Q2, quaternion.identity);

        //        Datas[0] = data;
        //    }
        //}

        //protected struct AngularConstraintData
        //{
        //    public quaternion q1, q2;
        //    public float3x3 IBodyInv1, IBodyInv2;

        //    public AngularConstraintData(Rigid b1, Rigid b2)
        //    {
        //        q1 = b1.Rotation;
        //        q2 = b2.Rotation;
        //        IBodyInv1 = b1.InertiaBodyInv;
        //        IBodyInv2 = b2.InertiaBodyInv;
        //    }
        //}

        //[BurstCompile]
        //protected struct AngularConstraintJob : IJob
        //{
        //    public float3 dq;
        //    public float angle;
        //    public float compliance;
        //    public float dt;

        //    public NativeArray<AngularConstraintData> Datas;

        //    public void Execute()
        //    {
        //        AngularConstraintData data = Datas[0];
        //        float C = math.length(dq) - angle;
        //        if (C == 0f)
        //            return;

        //        float3 n = math.normalize(dq);

        //        float3x3 I1Inv = math.mul(new float3x3(data.q1), math.mul(data.IBodyInv1, new float3x3(math.conjugate(data.q1))));
        //        float3x3 I2Inv = math.mul(new float3x3(data.q2), math.mul(data.IBodyInv2, new float3x3(math.conjugate(data.q2))));

        //        float w1 = math.mul(n, math.mul(I1Inv, n));
        //        float w2 = math.mul(n, math.mul(I2Inv, n));

        //        float alpha = compliance / (dt * dt);
        //        float dlambda = -C / (w1 + w2 + alpha);
        //        float3 p = dlambda * n;

        //        float3 I1invP = math.mul(I1Inv, p); 
        //        float3 I2invP = math.mul(I2Inv, p);

        //        quaternion Q1 = data.q1.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), data.q1).value;
        //        quaternion Q2 = data.q2.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), data.q2).value;

        //        data.q1 = math.normalizesafe(Q1, quaternion.identity);
        //        data.q2 = math.normalizesafe(Q2, quaternion.identity);

        //        Datas[0] = data;
        //    }

        //    public void Execute1()
        //    {
        //        float C = math.length(dq) - angle;
        //        if (C == 0f)
        //            return;

        //        AngularConstraintData data = Datas[0];

        //        // n = dq / ||dq||
        //        float3 n = math.normalize(dq);
        //        // n1 = q1^-1 * n
        //        // n2 = q2^-1 * n
        //        float3 n1 = math.rotate(math.conjugate(data.q1), n);
        //        float3 n2 = math.rotate(math.conjugate(data.q2), n);

        //        float3x3 I1inv = data.IBodyInv1;
        //        float3x3 I2inv = data.IBodyInv2;

        //        // w + n^T * Iinv * n
        //        float w1 = math.mul(n1, math.mul(I1inv, n1));
        //        float w2 = math.mul(n2, math.mul(I2inv, n2));

        //        float alpha = compliance / (dt * dt);
        //        float dlambda = -C / (w1 + w2 + alpha);
        //        //float3 p = dlambda * n;

        //        // p in local body frame
        //        float3 p1 = dlambda * n1;
        //        float3 p2 = dlambda * n2;

        //        // Iinv * p
        //        float3 I1invP = math.mul(I1inv, p1);
        //        float3 I2invP = math.mul(I2inv, p2);

        //        // q_new = q + 0.5 * [(Iinv * p), 0] * q
        //        quaternion Q1 = data.q1.value + math.mul(new quaternion(0.5f * new float4(I1invP, 0f)), data.q1).value;
        //        quaternion Q2 = data.q2.value - math.mul(new quaternion(0.5f * new float4(I2invP, 0f)), data.q2).value;

        //        // normalize q
        //        data.q1 = math.normalizesafe(Q1, quaternion.identity);
        //        data.q2 = math.normalizesafe(Q2, quaternion.identity);

        //        Datas[0] = data;
        //    }
        //}
    }
}

