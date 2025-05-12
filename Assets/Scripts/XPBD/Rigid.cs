using UnityEngine;
using Unity.Mathematics;

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
    [RequireComponent(typeof(Rigidbody))]
    public class Rigid : Body
    {
        public Rigidbody m_rigidbody { get; private set; }
        public Collider m_Collider { get; private set; }
        //public bool Fixed = false;
        public REAL3 Position { get; set; }


        // Since there could be intial rotation for some object
        // We seperate the world rotation into PhysicsRotation * InitialRotation
        // So that we get the PhysicsRotation always has a initial value of quaternion.identity, better for initializing joint constraints

        // Physics rotation
        public quaternion Rotation { get; set; }
        // Initial rotation
        private quaternion q0;

        // World rotation in unity world space
        private quaternion TotalRotation => math.mul(Rotation, q0);

        // Linear velocity
        public REAL3 vel { get; set; }

        // Angular velocity
        public REAL3 omega { get; set; }
        
        // Linear force
        public REAL3 Fext { get; set; }

        // Angular force (torque)
        public REAL3 Tau { get; set; }

        // Inertia and inverse inertia in the local body frame
        public REAL3x3 InertiaBody { get; private set; }
        public REAL3x3 InertiaBodyInv { get; private set; }
        public REAL3x3 InertiaInv
        {
            get
            {
                if (isFixed)
                    return 0;
                return math.mul(new float3x3(Rotation), math.mul(InertiaBodyInv, new float3x3(math.conjugate(Rotation))));
            }
        }
        // Previous position and rotation
        REAL3 prevPos;
        quaternion prevRot;

        // Initial velocity
        [SerializeField] 
        private REAL3 v0 = REAL3.zero;
        [SerializeField]
        private REAL3 omega0;

        #region Body
        public override void ClearCollision()
        {
        }

        public override void PreSolve(REAL dt, REAL3 gravity)
        {
            if (isGrabbed)
                return;
            if (isFixed) return;
            // Inverse mass is 0, which means it's fixed
            if (InvMass < Util.EPSILON)
            {
                vel = REAL3.zero;
                omega = REAL3.zero;
                return;
            }
                

            REAL3 g = (UseGravity) ? gravity : REAL3.zero;

            prevPos = Position;
            vel += dt * InvMass * Fext;
            vel += dt * g;
            Position += dt * vel;

            REAL3x3 I = math.mul(math.mul(new float3x3(Rotation), InertiaBody), new float3x3(math.conjugate(Rotation)));
            REAL3x3 Iinv = math.mul(math.mul(new float3x3(Rotation), InertiaBodyInv), new float3x3(math.conjugate(Rotation)));

            prevRot = Rotation;
            omega += dt * math.mul(Iinv, Tau - math.cross(omega, math.mul(I, omega)));
            quaternion Omega = new quaternion((float)omega.x, (float)omega.y, (float)omega.z, 0f);
            Omega.value *= 0.5f * (float)dt;
            quaternion dq = math.mul(Omega, Rotation);
            Rotation = math.normalizesafe(Rotation.value + dq.value, quaternion.identity);

        }

        public override void Solve(REAL dt)
        {
            if (isGrabbed)
                return;
            if (isFixed) return;
        }

        public override void PostSolve(REAL dt)
        {
            if (isGrabbed)
                return;
            if (isFixed) return;
            vel = (Position - prevPos) / dt;
            quaternion dq = math.mul(Rotation, math.conjugate(prevRot));
            REAL4 dqf = 2 * (REAL4)dq.value / dt;

            omega = new REAL3(dqf.x, dqf.y, dqf.z);
            if (dqf.w < 0f) omega *= -1;
             
        }
        public override void EndFrame()
        {
            ClearForce();
            //rigidCollisions.Clear();
            transform.SetPositionAndRotation((float3)Position, TotalRotation);
        }
        #endregion

        #region MonoBehaviour
        private void Awake()
        {
            Initialized();
        }
        private void OnEnable()
        {
            Simulation.get.AddBody(this);
        }

        private void Start()
        {
            isStarted = true;
        }

        private void OnDrawGizmos()
        {
            if (!isStarted)
                return;
        }

        #endregion

        private void Initialized()
        {
            bodyType = BodyType.Rigid;
            Transform transform = this.transform;
            m_rigidbody = GetComponent<Rigidbody>();
            m_rigidbody.mass = (float)mass;

            m_Collider = GetComponent<Collider>();

            Position = prevPos = (float3)transform.position;
            q0 = transform.rotation;
            Rotation = prevRot = Quaternion.identity;
            vel = v0;
            omega = omega0;
            Fext = REAL3.zero;
            Tau = REAL3.zero;

            REAL3 inertiaVec = (float3)m_rigidbody.inertiaTensor;
            InertiaBody = new(
                inertiaVec.x, 0, 0,
                0, inertiaVec.y, 0,
                0, 0, inertiaVec.z);
            InertiaBody = math.mul(math.mul(new float3x3(q0), InertiaBody), new float3x3(math.conjugate(q0)));
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
            Fext = REAL3.zero;
            Tau = REAL3.zero;
        }

        #region IGrabbable
        public override void StartGrab(REAL3 grabPos)
        {
            isGrabbed = true;

            Position = grabPos;

            vel = REAL3.zero;
            omega = REAL3.zero;
        }

        public override void MoveGrabbed(REAL3 grabPos)
        {
            Position = grabPos;
        }

        public override void EndGrab(REAL3 grabPos, REAL3 vel)
        {
            isGrabbed = false;

            Position = grabPos;
            this.vel = vel;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            hit = null;
            if(Intersection.IsRayHittingCollider(ray, m_Collider, out REAL hitDistance))
            {
                hit = new CustomHit(hitDistance, REAL3.zero, REAL3.zero);
            }
        }

        public override REAL3 GetGrabbedPos()
        {
            return Position;
        }
        #endregion
    }
}

