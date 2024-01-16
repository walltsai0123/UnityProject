using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
}
