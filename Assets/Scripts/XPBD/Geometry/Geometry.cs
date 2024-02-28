using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public abstract class Geometry : MonoBehaviour
    {
        public abstract Vector3 ClosestSurfacePoint(Vector3 point, out Vector3 surfaceNormal);
        public abstract bool IsInside(Vector3 point);
        public abstract bool Raycast(Ray ray, out RaycastHit hit, float maxDistance);
    }
}

