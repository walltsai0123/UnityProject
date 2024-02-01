using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class Intersection
{
    public static bool IsRayHittingCollider(Ray ray, Collider collider, out float hitDistance)
    {
        if(collider.Raycast(ray, out RaycastHit hit, float.MaxValue))
        {
            hitDistance = hit.distance;
            return true;
        }

        hitDistance = 0f;
        return false;
    }


    public static bool IsRayHittingMesh(Ray ray, float3 [] vertices, int[] triangles, out CustomHit bestHit)
    {
        bestHit = null;

        float smallestDistance = float.MaxValue;

        //Loop through all triangles and find the one thats the closest
        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 a = vertices[triangles[i + 0]];
            Vector3 b = vertices[triangles[i + 1]];
            Vector3 c = vertices[triangles[i + 2]];

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
    public static bool IsRayHittingPlane(Ray ray, Vector3 planeNormal, Vector3 planePos, out float t)
    {
        //Add default because have to
        t = 0f;

        //First check if the plane and the ray are perpendicular
        float NdotRayDirection = Vector3.Dot(planeNormal, ray.direction);

        //If the dot product is almost 0 then ray is perpendicular to the triangle, so no itersection is possible
        if (Mathf.Abs(NdotRayDirection) < Util.EPSILON)
        {
            return false;
        }

        //Compute d parameter using equation 2 by picking any point on the plane
        float d = -Vector3.Dot(planeNormal, planePos);

        //Compute t (equation 3)
        t = -(Vector3.Dot(planeNormal, ray.origin) + d) / NdotRayDirection;

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
    public static bool IsRayHittingTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Ray ray, out CustomHit hit)
    {
        hit = null;

        //Compute plane's normal
        Vector3 v0v1 = v1 - v0;
        Vector3 v0v2 = v2 - v0;

        //No need to normalize
        Vector3 planeNormal = Vector3.Cross(v0v1, v0v2);

        //
        // Step 1: Finding P (the intersection point) by turning the triangle into a plane
        //
        if (!IsRayHittingPlane(ray, planeNormal, v0, out float t))
        {
            return false;
        }

        //Compute the intersection point using equation 1
        Vector3 P = ray.origin + t * ray.direction;


        //
        // Step 2: inside-outside test
        //

        //Vector perpendicular to triangle's plane
        Vector3 C;

        //Edge 0
        Vector3 edge0 = v1 - v0;
        Vector3 vp0 = P - v0;

        C = Vector3.Cross(edge0, vp0);

        //P is on the right side 
        if (Vector3.Dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //Edge 1
        Vector3 edge1 = v2 - v1;
        Vector3 vp1 = P - v1;

        C = Vector3.Cross(edge1, vp1);

        //P is on the right side 
        if (Vector3.Dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //Edge 2
        Vector3 edge2 = v0 - v2;
        Vector3 vp2 = P - v2;

        C = Vector3.Cross(edge2, vp2);

        //P is on the right side 
        if (Vector3.Dot(planeNormal, C) < 0f)
        {
            return false;
        }

        //This ray hits the triangle

        //Calculate the custom data we need
        hit = new CustomHit(t, P, planeNormal.normalized);

        return true;
    }
}
