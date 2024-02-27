using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

namespace XPBD
{
    [RequireComponent(typeof(Rigidbody))]
    public class Rigid : Body
    {
        public enum eGeometryType { kSphere, kBox, kCylinder, kNone };
        public Rigidbody m_rigidbody { get; private set; }
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


        #region Body
        public override void ClearCollision()
        {
        }

        public override void PreSolve(float dt, Vector3 gravity)
        {
            if (isGrabbed)
                return;

            float3 g = (UseGravity) ? gravity : float3.zero;

            prevPos = Position;
            vel += dt * InvMass * fext;
            vel += dt * g;
            Position += dt * vel;

            float3x3 I = math.mul(math.mul(new float3x3(Rotation), InertiaBody), new float3x3(math.conjugate(Rotation)));
            float3x3 Iinv = math.mul(math.mul(new float3x3(Rotation), InertiaBodyInv), new float3x3(math.conjugate(Rotation)));

            prevRot = Rotation;
            omega += dt * math.mul(Iinv, Tau - math.cross(omega, math.mul(I, omega)));
            quaternion Omega = new quaternion(omega.x, omega.y, omega.z, 0f);
            Omega.value *= 0.5f * dt;
            quaternion dq = math.mul(Omega, Rotation);
            Rotation = math.normalize(Rotation.value + dq.value);

            ClearForce();
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
            
            //rigidCollisions.Clear();
            transform.SetPositionAndRotation(Position, Rotation);
        }
        #endregion

        #region MonoBehaviour
        private void Awake()
        {
            Initialized();
            Debug.Log("Rigid Awake");
        }

        private void Start()
        {
            Simulation.get.AddBody(this);
            isStarted = true;
        }

        private void OnDrawGizmos()
        {
            if (!isStarted)
                return;
            //Collider collider = GetComponent<Collider>();
            //Bounds bounds = collider.bounds;
            //bounds.Expand(2f * math.length(vel) * 0.02f);
            //Gizmos.color = Color.red;
            //Gizmos.DrawWireCube(collider.bounds.center, collider.bounds.size);
            //Gizmos.color = Color.blue;
            //Gizmos.DrawWireCube(bounds.center, bounds.size);
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

