using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
public static class Util
{
    public readonly static float EPSILON = 1e-10f;
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

    public static void SetLayerRecursive(GameObject gameObject, int layer)
    {
        gameObject.layer = layer;

        foreach (Transform child in gameObject.transform)
        {
            SetLayerRecursive(child.gameObject, layer);
        }
    }

    public static Vector3 GetPerpendicularVector(Vector3 v)
    {
        if (v.magnitude < EPSILON)
            return Vector3.zero;

        Vector3 A = v.normalized;
        Vector3 B = Vector3.Cross(A, Vector3.right);

        // check if A and B are parellel
        // if parallel, do again with (0, 0, -1)
        if (math.length(B) < math.EPSILON)
        {
            B = Vector3.Cross(A, Vector3.back);
        }
        B = math.normalize(B);

        return B;
    }
}

