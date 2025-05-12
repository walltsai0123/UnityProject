using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
    public abstract class Body : MonoBehaviour, IGrabbable
    {
        public enum BodyType
        {
            None,
            Rigid,
            Soft
        };
        //public int ID;
        [DebugOnly]public BodyType bodyType = BodyType.None;
        public REAL mass = 1f;
        public bool isFixed = false;
        public bool UseGravity = true;
        public bool EnableContact = true;
        public bool grabbable = true;

        protected bool isGrabbed = false;
        protected bool isStarted = false;
        
        public REAL InvMass
        {
            get 
            {
                if (mass == 0f || isFixed)
                    return 0f;
                return 1f / mass; 
            }
        }

        public virtual void ClearCollision()
        {
            Debug.Log("Body ClearCollision");
        }

        public virtual void PreSolve(REAL dt, REAL3 gravity)
        {
            Debug.Log("Body PreSolve");
        }
        public virtual void Solve(REAL dt)
        {
            Debug.Log("Body Solve");
        }
        public virtual void PostSolve(REAL dt)
        {
            Debug.Log("Body PostSolve");
        }

        public virtual void EndFrame()
        {
            Debug.Log("Body EndFrame");
        }

        // IGrabbable methods
        public bool Grabbable => grabbable;
        public abstract void StartGrab(REAL3 grabPos);

        public abstract void MoveGrabbed(REAL3 grabPos);

        public abstract void EndGrab(REAL3 grabPos, REAL3 vel);

        public abstract void IsRayHittingBody(Ray ray, out CustomHit hit);

        public abstract REAL3 GetGrabbedPos();
    }
}

