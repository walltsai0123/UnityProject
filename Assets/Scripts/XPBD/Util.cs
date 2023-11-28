using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class Util
    {
        public static bool IsInsideCollider(Collider collider, float3 point)
        {
            float3 center;
            float3 direction;
            Ray ray;
            bool hit;

            // Use collider bounds to get the center of the collider. May be inaccurate
            // for some colliders (i.e. MeshCollider with a 'plane' mesh)
            center = collider.bounds.center;

            // Cast a ray from point to center
            direction = center - point;
            ray = new Ray(point, direction);
            hit = collider.Raycast(ray, out _, math.length(direction));

            // If we hit the collider, point is outside. So we return !hit
            return !hit;
        }
    }
}
