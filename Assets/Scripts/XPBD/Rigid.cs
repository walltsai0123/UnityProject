using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    [RequireComponent(typeof(Rigidbody))]
    public class Rigid : Body
    {
        public enum eGeometryType { kSphere, kBox, kCylinder, kNone };
        private Rigidbody m_rigidbody;
        //public bool Fixed = false;
        public float3 Position { get; set; }
        public quaternion Rotation { get; set; }
        public float3 vel { get; private set; }
        public float3 omega { get; private set; }
        public float3 fext { get; private set; }
        public float3 Tau { get; set; }

        public float3x3 InertiaBody { get; private set; }
        public float3x3 InertiaBodyInv { get; private set; }

        // Previous position and rotation
        private float3 prevPos;
        private quaternion prevRot;
       
        public eGeometryType geometryType = eGeometryType.kNone;

        public override void CollectCollision(float dt)
        {
        }

        public override void PreSolve(float dt, Vector3 gravity)
        {
            if(UseGravity)
            {
                float3 g = gravity;
                fext += mass * g;
            }
            //Tau = float3.zero;

            prevPos = Position;
            vel += dt * InvMass * fext;
            Position += dt * vel;

            float3x3 I = math.mul(math.mul(new float3x3(Rotation), InertiaBody), new float3x3(math.conjugate(Rotation)));
            float3x3 Iinv = math.mul(math.mul(new float3x3(Rotation), InertiaBodyInv), new float3x3(math.conjugate(Rotation)));

            prevRot = Rotation;
            omega += dt * math.mul(Iinv, Tau - math.cross(omega, math.mul(I, omega)));
            quaternion Omega = new quaternion(omega.x, omega.y, omega.z, 0f);
            Omega.value *= 0.5f * dt;
            quaternion dq = math.mul(Omega, Rotation);
            Rotation = math.normalize(Rotation.value + dq.value);
        }

        public override void Solve(float dt)
        {
            
        }

        public override void PostSolve(float dt)
        {
            vel = (Position - prevPos) / dt;
            quaternion dq = math.mul(Rotation, math.conjugate(prevRot));
            float4 dqf = 2.0f * dq.value / dt;

            omega = new float3(dqf.x, dqf.y, dqf.z);
            if (dqf.w < 0f) omega *= -1;
        }
        public override void VelocitySolve(float dt)
        {
            
        }
        public override void EndFrame()
        {
            ClearForce();
            transform.SetPositionAndRotation(Position, Rotation);
        }

        private void Awake()
        {
            //if(geometryType == eGeometryType.kNone)
            //{
            //    Debug.Log("geometry unset");
            //    return;
            //}
            //Transform m_transform = this.transform;

            //if (geometryType == eGeometryType.kBox)
            //{
            //    BoxCollider box = GetComponent<BoxCollider>();
            //    if(box == null)
            //    {
            //        Debug.Log("BoxCollider not found");
            //        return;
            //    }
            //    Vector3 boxSize = box.size;
            //    boxSize.x *= transform.localScale.x;
            //    boxSize.y *= transform.localScale.y;
            //    boxSize.z *= transform.localScale.z;
            //    ID = BackEnd.AddXPBDRigidBox(m_transform.position, m_transform.rotation, boxSize, mass);
            //}
            //else if (geometryType == eGeometryType.kCylinder)
            //{
            //    float radius = 0.5f * transform.localScale.x;
            //    float height = 2f * transform.localScale.y;
            //    ID = BackEnd.AddXPBDRigidCylinder(m_transform.position, m_transform.rotation, radius, height, mass);
                
            //}

            Initialized();
            
            //Debug.Log(ID);
            Debug.Log("Rigid Awake");
        }

        private void Start()
        {
            Simulation.get.AddBody(this);
        }

        private void Initialized()
        {
            Transform transform = this.transform;
            m_rigidbody = GetComponent<Rigidbody>();
            m_rigidbody.mass = mass;

            Position = prevPos = transform.position;
            Rotation = prevRot = transform.rotation;
            vel = float3.zero;
            omega = float3.zero;
            //omega = new(1, 0, 0);
            fext = float3.zero;
            Tau = float3.zero;

            float3 inertiaVec = m_rigidbody.inertiaTensor;
            InertiaBody = new(
                inertiaVec.x, 0, 0,
                0, inertiaVec.y, 0,
                0, 0, inertiaVec.z);
            InertiaBodyInv = math.inverse(InertiaBody);
            if(mass == 0f)
            {
                InertiaBody *= 0f;
                InertiaBodyInv *= 0f;
            }

            m_rigidbody.isKinematic = true;

            // Debug.Log("InertiaBody: " + InertiaBody);
        }

        private void ClearForce()
        {
            fext = float3.zero;
            Tau = float3.zero;
        }
    }
}

