using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
public static class Util
{
    public readonly static float EPSILON = 1e-6f;
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

    private static void LUDecomposition(float2x2 A, out float2x2 L, out float2x2 U, out int2 P)
    {
        int n = 2;
        L = float2x2.zero;
        U = float2x2.zero;
        P = int2.zero;

        for (int i = 0; i < n; i++)
            P[i] = i;

        // Partial Pivoting
        if (math.abs(A[0][1]) > Math.Abs(A[0][0]))
        {
            // Swap rows in permutation array
            int temp = P[0];
            P[0] = P[1];
            P[1] = temp;

            // Swap rows in A
            for (int i = 0; i < n; i++)
            {
                float tempA = A[i][0];
                A[i][0] = A[i][1];
                A[i][1] = tempA;
            }
        }

        // U matrix is upper triangular
        U[0][0] = A[0][0];
        U[1][0] = A[1][0];
        U[0][1] = 0;
        U[1][1] = A[1][1] - (A[1][0] * A[0][1] / A[0][0]);

        // L matrix is lower triangular
        L[0][0] = 1;
        L[1][0] = 0;
        L[0][1] = A[0][1] / A[0][0];
        L[1][1] = 1;
    }

    public static float2 LUSolve(float2x2 matrix, float2 b)
    {
        int n = 2;
        float2 y = new float2();
        float2 x = new float2();
        float2 Pb = new float2();

        LUDecomposition(matrix, out float2x2 L, out float2x2 U, out int2 P);

        // Apply permutation to b
        for (int i = 0; i < n; i++)
        {
            Pb[i] = b[P[i]];
        }

        // Forward substitution to solve Ly = b

        y[0] = Pb[0];
        y[1] = Pb[1] - L[0][1] * y[0];

        // Backward substitution to solve Ux = y
        
        x[1] = y[1] / U[1][1];
        x[0] = (y[0] - U[1][0] * x[1]) / U[0][0];

        return x;
    }
}

