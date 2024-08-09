using Unity.Mathematics;
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
    public class Plane : Geometry
    {
        // We assume all the planes has the same attributes
        public REAL3 normal = new(0,1,0);
        public REAL3 center = new(0,0,0);
        public REAL2 size = new(10, 10);

        public override REAL3 ClosestSurfacePoint(REAL3 point, out REAL3 surfaceNormal)
        {
            // transform to local
            REAL3 local = (float3)transform.InverseTransformPoint((float3)point);

            REAL t = -math.dot(normal, local - center);
            REAL3 result = local + t * normal;

            // transform back to world
            result = (float3)transform.TransformPoint((float3)result);
            surfaceNormal = (float3)transform.TransformDirection((float3)normal);
            return result;
        }
        public override bool IsInside(REAL3 point)
        {
            REAL3 local = (float3)transform.InverseTransformPoint((float3)point);

            // dp < 0 => under plane
            REAL dp = math.dot(local - center, normal);
            REAL3 tangent = (local - center) - dp * normal;

            if (dp <= Util.EPSILON)
                if (math.abs(tangent.x) <= size.x / 2f && math.abs(tangent.z) <= size.y / 2f)
                    return true;

            return false;
        }
        public override bool Raycast(Ray ray, out RaycastHit hit, float maxDistance)
        {
            hit = new();

            //Add default because have to
            REAL t = 0f;

            //First check if the plane and the ray are perpendicular
            REAL NdotRayDirection = math.dot(normal, (float3)ray.direction);

            //If the dot product is almost 0 then ray is perpendicular to the triangle, so no itersection is possible
            if (math.abs(NdotRayDirection) < Util.EPSILON)
            {
                return false;
            }

            //Compute d parameter using equation 2 by picking any point on the plane
            REAL d = -math.dot(normal, center);

            //Compute t (equation 3)
            t = -(math.dot(normal, (float3)ray.origin) + d) / NdotRayDirection;

            //Check if the plane is behind the ray
            if (t < 0)
            {
                return false;
            }

            //Check if t is out of range
            if (t > maxDistance)
                return false;

            hit.normal = (float3)normal;
            hit.distance = (float)t;
            hit.point = ray.GetPoint((float)t);

            return true;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(transform.position, new Vector3(transform.lossyScale.x * (float)size.x, 0f, transform.lossyScale.z * (float)size.y));
        }
    }
}

