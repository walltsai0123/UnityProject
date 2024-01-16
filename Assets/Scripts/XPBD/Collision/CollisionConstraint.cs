using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace XPBD
{
    public abstract class CollisionConstraint
    {
        public abstract void SolveCollision(float dt);

        public abstract void VelocitySolve(float dt);
    }
}

