using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;


namespace XPBD
{
    public abstract class CollisionConstraint
    {
        public float4 frictionCoef;
        public abstract void SolveCollision(float dt);

        public abstract void VelocitySolve(float dt);
    }
}

