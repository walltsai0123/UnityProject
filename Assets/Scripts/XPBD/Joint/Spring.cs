using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace XPBD
{
    public class Spring : Joint
    {
        public Vector3 anchor;
        public Vector3 distance;

        [SerializeField, Min(0f)]
        private float compliance = 0f;

        [SerializeField, Range(0f, 1f)]
        private float linearDamping = 0f;

        [SerializeField, Range(0f, 1f)]
        private float angularDamping = 0f;

        private float3 r1, r2;
        public override void SolveConstraint(float dt)
        {
            SolvePositionConstraint(dt);
        }


        public override void SolveVelocities(float dt)
        {
            float3 dv = (body2.vel - body1.vel) * math.min(linearDamping * dt, 1);
            float3 domega = (body2.omega - body1.omega) * math.min(angularDamping * dt, 1);

            // linear part
            float3 p = dv / (body1.InvMass + body2.InvMass);
            if(!body1.isFixed)
                body1.vel += p * body1.InvMass;
            if(!body2.isFixed)
                body2.vel -= p * body2.InvMass;

            // angular part
            float3 n = math.normalizesafe(domega, float3.zero);
            float w1 = math.mul(n, math.mul(body1.InertiaInv, n));
            float w2 = math.mul(n, math.mul(body2.InertiaInv, n));
            if(w1 + w2 < Util.EPSILON)
                return;
            p = domega / (w1 + w2);
            if (!body1.isFixed)
                body1.omega += math.mul(body1.InertiaInv, p);
            if (!body2.isFixed)
                body2.omega -= math.mul(body2.InertiaInv, p);
        }

        private void SolvePositionConstraint(float dt)
        {
            PositionConstraint positionConstraint = new PositionConstraint(body1, body2, r1, r2);

            float3 p1 = body1.Position + positionConstraint.r1;
            float3 p2 = body2.Position + positionConstraint.r2;
            float3 dist = distance;
            float3 delta_x = p1 - p2 - dist;

            float d_lambda = positionConstraint.GetDeltaLambda(dt, compliance, 0f, delta_x);
        }

        private void SolveAngularConstraint(float dt)
        {
            throw new NotImplementedException();
        }

        void Awake()
        {
            body1 = GetComponent<Rigid>();
        }
        void Start()
        {
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        void Initialize()
        {
            r1 = anchor;
            float3 Anchor = body1.Position + math.rotate(body1.Rotation, r1);
            r2 = math.rotate(math.conjugate(body2.Rotation), Anchor - body2.Position);
        }

        private void OnDrawGizmosSelected()
        {
            if (body2 == null)
                return;

            Vector3 R1 = transform.position + transform.rotation * anchor;
            Vector3 R2 = body2.transform.position + body2.transform.rotation * r2;


            Gizmos.color = Color.green;
            Gizmos.DrawSphere(R1, 0.1f);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(R2, 0.1f);

            Gizmos.DrawLine(R1, R2);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(R1, R1 - distance);
        }
    }
}
