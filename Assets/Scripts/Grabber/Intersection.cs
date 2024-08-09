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
public class Intersection
{
    public static bool IsRayHittingCollider(Ray ray, Collider collider, out REAL hitDistance)
    {
        if(collider.Raycast(ray, out RaycastHit hit, float.MaxValue))
        {
            hitDistance = hit.distance;
            return true;
        }

        hitDistance = 0f;
        return false;
    }


    public static bool IsRayHittingMesh(Ray ray, REAL3 [] vertices, int[] triangles, out CustomHit bestHit)
    {
        bestHit = null;

        REAL smallestDistance = REAL.MaxValue;

        //Loop through all triangles and find the one thats the closest
        for (int i = 0; i < triangles.Length; i += 3)
        {
            REAL3 a = vertices[triangles[i + 0]];
            REAL3 b = vertices[triangles[i + 1]];
            REAL3 c = vertices[triangles[i + 2]];

            if (IsRayHittingTriangle(a, b, c, ray, out CustomHit hit))
            {
                if (hit.distance < smallestDistance)
                {
                    smallestDistance = hit.distance;

                    bestHit = hit;

                    bestHit.index = i;
                }
            }
        }

        //If at least a triangle was hit
        bool hitMesh = false;

        if (bestHit != null)
        {
            hitMesh = true;
        }

        return hitMesh;
    }
    //
    // Ray-plane intersection
    //

    //https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
    public static bool IsRayHittingPlane(Ray ray, REAL3 planeNormal, REAL3 planePos, out REAL t)
    {
        //Add default because have to
        t = 0f;

        //First check if the plane and the ray are perpendicular
        REAL NdotRayDirection = math.dot(planeNormal, (float3)ray.direction);

        //If the dot product is almost 0 then ray is perpendicular to the triangle, so no itersection is possible
        if (math.abs(NdotRayDirection) < Util.EPSILON)
        {
            return false;
        }

        //Compute d parameter using equation 2 by picking any point on the plane
        REAL d = -math.dot(planeNormal, planePos);

        //Compute t (equation 3)
        t = -(math.dot(planeNormal, (float3)ray.origin) + d) / NdotRayDirection;

        //Check if the plane is behind the ray
        if (t < 0)
        {
            return false;
        }

        return true;
    }



    //
    // Ray-triangle intersection
    //

    //First do ray-plane itersection and then check if the itersection point is within the triangle
    //https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
    public static bool IsRayHittingTriangle(REAL3 v0, REAL3 v1, REAL3 v2, Ray ray, out CustomHit hit)
    {
        hit = null;

        //Compute plane's normal
        REAL3 v0v1 = v1 - v0;
        REAL3 v0v2 = v2 - v0;

        //No need to normalize
        REAL3 planeNormal = math.cross(v0v1, v0v2);

        //
        // Step 1: Finding P (the intersection point) by turning the triangle into a plane
        //
        if (!IsRayHittingPlane(ray, planeNormal, v0, out REAL t))
        {
            return false;
        }

        //Compute the intersection point using equation 1
        REAL3 rayDir = (float3)ray.direction;
        REAL3 P = (float3)ray.origin + t * rayDir;

        //
        // Step 2: inside-outside test
        //

        //REAL perpendicular to triangle's plane
        REAL3 C;

        //Edge 0
        REAL3 edge0 = v1 - v0;
        REAL3 vp0 = P - v0;

        C = math.cross(edge0, vp0);

        //P is on the right side 
        if (math.dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //Edge 1
        REAL3 edge1 = v2 - v1;
        REAL3 vp1 = P - v1;

        C = math.cross(edge1, vp1);

        //P is on the right side 
        if (math.dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //Edge 2
        REAL3 edge2 = v0 - v2;
        REAL3 vp2 = P - v2;

        C = math.cross(edge2, vp2);

        //P is on the right side 
        if (math.dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //This ray hits the triangle

        //Calculate the custom data we need
        hit = new CustomHit(t, P, math.normalize(planeNormal));

        return true;
    }
}
