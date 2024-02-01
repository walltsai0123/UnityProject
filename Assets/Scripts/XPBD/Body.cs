using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        public BodyType bodyType = BodyType.None;
        public float mass = 1f;
        public bool UseGravity = true;
        public bool EnableContact = true;
        protected bool isGrabbed = false;
        public bool grabbable = true;
        protected bool isStarted = false;
        
        public float InvMass
        {
            get 
            {
                if (mass == 0f)
                    return 0f;
                return 1f / mass; 
            }
        }

        public virtual void ClearCollision()
        {
            Debug.Log("Body ClearCollision");
        }

        public virtual void PreSolve(float dt, Vector3 gravity)
        {
            Debug.Log("Body PreSolve");
        }
        public virtual void Solve(float dt)
        {
            Debug.Log("Body Solve");
        }
        public virtual void PostSolve(float dt)
        {
            Debug.Log("Body PostSolve");
        }
        public virtual void VelocitySolve(float dt)
        {
            Debug.Log("Body Velocity");
        }
        public virtual void EndFrame()
        {
            Debug.Log("Body EndFrame");
        }

        // IGrabbable methods
        public bool Grabbable => grabbable;
        public abstract void StartGrab(Vector3 grabPos);

        public abstract void MoveGrabbed(Vector3 grabPos);

        public abstract void EndGrab(Vector3 grabPos, Vector3 vel);

        public abstract void IsRayHittingBody(Ray ray, out CustomHit hit);

        public abstract Vector3 GetGrabbedPos();
    }
}

