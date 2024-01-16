using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

namespace XPBD
{
    [RequireComponent(typeof(Rigidbody))]
    public class Rigid : Body
    {
        public enum eGeometryType { kSphere, kBox, kCylinder, kNone };
        private Rigidbody m_rigidbody;
        public Collider m_Collider { get; private set; }
        //public bool Fixed = false;
        public float3 Position { get; set; }
        public quaternion Rotation { get; set; }
        public float3 vel { get; set; }
        public float3 omega { get; set; }
        public float3 fext { get; set; }
        public float3 Tau { get; set; }

        public float3x3 InertiaBody { get; private set; }
        public float3x3 InertiaBodyInv { get; private set; }

        // Previous position and rotation
        public float3 prevPos { get; private set; }
        public quaternion prevRot { get; private set; }

        [SerializeField] 
        private Vector3 v0 = Vector3.zero;

        public eGeometryType geometryType = eGeometryType.kNone;

        private List<ContactPoint> contactPoints;

        public List<RigidCollision> rigidCollisions;

        private float grabMass;

        #region Body
        public override void CollectCollision(float dt, Primitive primitive)
        {
        }

        public override void PreSolve(float dt, Vector3 gravity)
        {
            if (isGrabbed)
                return;
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
            if (isGrabbed)
                return;
        }

        public override void PostSolve(float dt)
        {
            if (isGrabbed)
                return;

            vel = (Position - prevPos) / dt;
            quaternion dq = math.mul(Rotation, math.conjugate(prevRot));
            float4 dqf = 2.0f * dq.value / dt;

            omega = new float3(dqf.x, dqf.y, dqf.z);
            if (dqf.w < 0f) omega *= -1;
        }
        public override void VelocitySolve(float dt)
        {
            if (isGrabbed)
                return;
        }
        public override void EndFrame()
        {
            ClearForce();
            rigidCollisions.Clear();
            transform.SetPositionAndRotation(Position, Rotation);
        }
        #endregion

        #region MonoBehaviour
        private void Awake()
        {
            Initialized();
            contactPoints = new();
            rigidCollisions = new();
            Debug.Log("Rigid Awake");
        }

        private void Start()
        {
            Simulation.get.AddBody(this);
        }

        private void OnCollisionEnter(Collision collision)
        {
            AddCollision(collision);
        }
        private void OnCollisionStay(Collision collision)
        {
            AddCollision(collision);
        }

        private void OnDrawGizmosSelected()
        {
            if (contactPoints == null)
                return;
            foreach (var cp in contactPoints)
            {
                Vector3 point = cp.point;
                Vector3 normal = cp.normal;

                Gizmos.DrawSphere(point, 0.05f);
                Gizmos.DrawRay(point, normal);
            }
        }

        #endregion

        private void Initialized()
        {
            bodyType = BodyType.Rigid;
            Transform transform = this.transform;
            m_rigidbody = GetComponent<Rigidbody>();
            m_rigidbody.mass = mass;

            m_Collider = GetComponent<Collider>();

            Position = prevPos = transform.position;
            Rotation = prevRot = transform.rotation;
            vel = v0;
            omega = float3.zero;
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
        }

        private void ClearForce()
        {
            fext = float3.zero;
            Tau = float3.zero;
        }

        private void AddCollision(Collision collision)
        {
            if (isGrabbed)
                return;
            if (collision.collider.gameObject.CompareTag("Primitive"))
            {
                collision.GetContacts(contactPoints);

                Primitive primitive = collision.collider.gameObject.GetComponent<Primitive>();

                //rigidCollisions.Add(new(this, primitive, contactPoints[0]));
                foreach (var cp in contactPoints)
                {
                    rigidCollisions.Add(new(this, primitive, cp));
                }
            }
        }

        #region IGrabbable
        public override void StartGrab(Vector3 grabPos)
        {
            isGrabbed = true;

            Position = grabPos;

            vel = Vector3.zero;
            omega = Vector3.zero;
        }

        public override void MoveGrabbed(Vector3 grabPos)
        {
            Position = grabPos;
        }

        public override void EndGrab(Vector3 grabPos, Vector3 vel)
        {
            isGrabbed = false;

            Position = grabPos;
            this.vel = vel;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            hit = null;
            if(Intersection.IsRayHittingCollider(ray, m_Collider, out float hitDistance))
            {
                hit = new CustomHit(hitDistance, Vector3.zero, Vector3.zero);
            }
        }

        public override Vector3 GetGrabbedPos()
        {
            return Position;
        }
        #endregion
    }
}

