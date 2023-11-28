using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    [RequireComponent(typeof(SoftBody))]
    public class Attach : Constraint
    {
        private SoftBody thisBody;
        public Rigid attachedBody;

        // public int ID1, ID2;

        private struct ParitilcPos
        {
            public int i;
            public float3 pos;
            public ParitilcPos(int index, float3 Pos)
            {
                i = index;
                pos = Pos;
            }
        }
        private List<ParitilcPos> paritilcPos;

        public override void SolveConstraint(float dt)
        {
            foreach (ParitilcPos pPos in paritilcPos)
            {
                float3 clocal = math.rotate(math.conjugate(attachedBody.Rotation), thisBody.Pos[pPos.i] - attachedBody.Position);
                float3 dr = clocal - pPos.pos;
                float C = math.length(dr);

                if (C < math.EPSILON)
                    continue;

                float w1 = 1f / thisBody.mass;
                float w2 = attachedBody.InvMass;
                float alpha = 0.0f;
                float dlambda = -C / (w1 + w2 + alpha);
                float3 p = dlambda * math.normalize(dr);
                p = math.rotate(attachedBody.Rotation, p);

                thisBody.Pos[pPos.i] += p * w1;
                attachedBody.Position -= p * w2;
            }
        }

        private void Awake()
        {
            thisBody = GetComponent<SoftBody>();
            paritilcPos = new List<ParitilcPos>();

            Debug.Log("Attach Awake");
        }
        private void Start()
        {
            //if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            //{
            //    ID1 = attachedBody.ID;
            //    ID2 = thisBody.ID;
            //    BackEnd.AttachRigidSoft(ID1, ID2);
            //}
            Initialized();
            Simulation.get.AddConstraints(this);
        }
        private void Initialized()
        {
            Collider collider = attachedBody.GetComponent<Collider>();
            if(collider == null)
            {
                Debug.LogWarning("Collider null");
                return;
            }
            Debug.Log(collider);
            for(int i = 0; i < thisBody.VerticesNum; ++i)
            {
                if (!Util.IsInsideCollider(collider, thisBody.Pos[i]))
                    continue;

                float3 clocal = math.rotate(math.conjugate(attachedBody.Rotation), thisBody.Pos[i] - attachedBody.Position);
                paritilcPos.Add(new ParitilcPos(i, clocal));
            }
        }
    }
    
}

