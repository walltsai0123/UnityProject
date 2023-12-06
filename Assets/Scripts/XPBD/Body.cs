using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public abstract class Body : MonoBehaviour
    {
        //public int ID;
        public float mass = 1f;
        public bool UseGravity = true;

        public float InvMass
        {
            get 
            {
                if (mass == 0f)
                    return 0f;
                return 1f / mass; 
            }
        }

        public virtual void CollectCollision(float dt)
        {
            Debug.Log("Body CollectCollision");
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
    }
}

