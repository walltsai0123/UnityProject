using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;

namespace XPBD
{
    [RequireComponent(typeof(Rigid))]
    public class FixedJoint : Constraint
    {
        private Rigid thisBody;
        public Rigid attachedBody;

        // public int ID1, ID2;

        private float3 anchor;
        private quaternion b1q0;
        private quaternion b2q0;
        private JobHandle jobHandle;

        public override void SolveConstraint(float dt)
        {
            //AngularConstraintJob angularConstraintJob = new AngularConstraintJob
            //{
            //    q1 = math.mul(math.conjugate(b1q0), thisBody.Rotation),
            //    q2 = math.mul(math.conjugate(b2q0), attachedBody.Rotation),
            //    IBodyInv1 = thisBody.InertiaBodyInv,
            //    IBodyInv2 = attachedBody.InertiaBodyInv,
            //    dq = 2f * math.mul(thisBody.Rotation, math.conjugate(attachedBody.Rotation)).value.xyz,
            //    angle = 0f,
            //    compliance = 0f,
            //    dt = dt
            //};
            ////Debug.Log("Before Job");
            ////Debug.Log("q1: " + angularConstraintJob.q1);
            ////Debug.Log("q2: " + angularConstraintJob.q2);

            //jobHandle = angularConstraintJob.Schedule();
            //jobHandle.Complete();

            ////Debug.Log("After Job");
            ////Debug.Log("q1: " + angularConstraintJob.q1);
            ////Debug.Log("q2: " + angularConstraintJob.q2);

            //thisBody.Rotation = math.mul(b1q0, angularConstraintJob.q1);
            //attachedBody.Rotation = math.mul(b2q0, angularConstraintJob.q2);

            NativeArray<PositionConstraintData> positionConstraintDatas = new NativeArray<PositionConstraintData>(1, Allocator.TempJob);
            positionConstraintDatas[0] = new PositionConstraintData(thisBody, attachedBody, anchor, float3.zero);

            PositionConstraintJob positionConstraintJob = new PositionConstraintJob
            {
                Datas = positionConstraintDatas,
                dmax = 0f,
                compliance = 0f,
                dt = dt
            };

            //Debug.Log("Before Job");
            //Debug.Log("x1: " + positionConstraintDatas[0].x1);
            //Debug.Log("x2: " + positionConstraintDatas[0].x2);

            jobHandle = positionConstraintJob.Schedule();
            jobHandle.Complete();
            //Debug.Log("After Job");
            //Debug.Log("x1: " + positionConstraintDatas[0].x1);
            //Debug.Log("x2: " + positionConstraintDatas[0].x2);

            thisBody.Position = positionConstraintDatas[0].x1;
            attachedBody.Position = positionConstraintDatas[0].x2;
            thisBody.Rotation = positionConstraintDatas[0].q1;
            attachedBody.Rotation = positionConstraintDatas[0].q2;

            positionConstraintDatas.Dispose();
        }

        private void Awake()
        {
            thisBody = GetComponent<Rigid>();
            Debug.Log("FixedJoint Awake");
        }
        void Start()
        {
            //if (thisBody.gameObject.activeInHierarchy && attachedBody.gameObject.activeInHierarchy)
            //{
            //    ID1 = thisBody.ID;
            //    ID2 = attachedBody.ID;

            //    BackEnd.AddFixedJoint(ID1, ID2);
            //}
            Initialize();
            Simulation.get.AddConstraints(this);
        }

        void Initialize()
        {
            b1q0 = thisBody.Rotation;
            b2q0 = attachedBody.Rotation;

            anchor = math.rotate(math.conjugate(b1q0), attachedBody.Position - thisBody.Position);
        }
    }
}

