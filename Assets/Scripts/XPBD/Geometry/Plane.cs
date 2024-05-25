using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace XPBD
{
    public class Plane : Geometry
    {
        // We assume all the planes has the same attributes
        public Vector3 normal = new(0,1,0);
        public Vector3 center = new(0,0,0);
        public Vector2 size = new(10, 10);

        public override Vector3 ClosestSurfacePoint(Vector3 point, out Vector3 surfaceNormal)
        {
            // transform to local
            Vector3 local = transform.InverseTransformPoint(point);

            float t = -Vector3.Dot(normal, local - center);
            Vector3 result = local + t * normal;

            // transform back to world
            result = transform.TransformPoint(result);
            surfaceNormal = transform.TransformDirection(normal);
            return result;
        }
        public override bool IsInside(Vector3 point)
        {
            Vector3 local = transform.InverseTransformPoint(point);

            // dp < 0 => under plane
            float dp = Vector3.Dot(local - center, normal);
            Vector3 tangent = (local - center) - dp * normal;

            if (dp <= 0f)
                if (Mathf.Abs(tangent.x) <= size.x / 2f && Mathf.Abs(tangent.z) <= size.y / 2f)
                    return true;

            return false;
        }
        public override bool Raycast(Ray ray, out RaycastHit hit, float maxDistance)
        {
            hit = new();

            //Add default because have to
            float t = 0f;

            //First check if the plane and the ray are perpendicular
            float NdotRayDirection = Vector3.Dot(normal, ray.direction);

            //If the dot product is almost 0 then ray is perpendicular to the triangle, so no itersection is possible
            if (Mathf.Abs(NdotRayDirection) < Util.EPSILON)
            {
                return false;
            }

            //Compute d parameter using equation 2 by picking any point on the plane
            float d = -Vector3.Dot(normal, center);

            //Compute t (equation 3)
            t = -(Vector3.Dot(normal, ray.origin) + d) / NdotRayDirection;

            //Check if the plane is behind the ray
            if (t < 0)
            {
                return false;
            }

            //Check if t is out of range
            if (t > maxDistance)
                return false;

            hit.normal = normal;
            hit.distance = t;
            hit.point = ray.GetPoint(t);

            return true;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(transform.position, new Vector3(size.x, 0f, size.y));
        }
    }
}

