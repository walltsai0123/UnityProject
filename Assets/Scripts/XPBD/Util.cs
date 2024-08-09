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

using UnityEngine;
using Unity.Mathematics;
using System;


public static class Util
{
#if USE_FLOAT
    public readonly static REAL EPSILON = 1E-6f;
#else
    public readonly static REAL EPSILON = 1E-10;
#endif
    public static REAL EPSILON_SQUARE
    {
        get => EPSILON * EPSILON;
    }
    public static bool IsInsideCollider(Collider collider, REAL3 point)
    {
        REAL3 center;
        REAL3 direction;
        Ray ray;
        bool hit;

        // Use collider bounds to get the center of the collider. May be inaccurate
        // for some colliders (i.e. MeshCollider with a 'plane' mesh)
        center = (float3)collider.bounds.center;

        // Cast a ray from point to center
        direction = center - point;
        ray = new Ray((float3)point, (float3)direction);
        hit = collider.Raycast(ray, out _, (float)math.length(direction));

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

    public static REAL3 GetPerpendicularVector(REAL3 v)
    {
        if (math.lengthsq(v) < EPSILON_SQUARE)
            return REAL3.zero;

        REAL3 A = math.normalize(v);
        REAL3 B = math.cross(A, (float3)Vector3.right);

        // check if A and B are parellel
        // if parallel, do again with (0, 0, -1)
        if (math.lengthsq(B) < EPSILON_SQUARE)
        {
            B = math.cross(A, (float3)Vector3.back);
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

    private static void LUDecomposition(double2x2 A, out double2x2 L, out double2x2 U, out int2 P)
    {
        int n = 2;
        L = double2x2.zero;
        U = double2x2.zero;
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
                double tempA = A[i][0];
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

    public static double2 LUSolve(double2x2 matrix, double2 b)
    {
        int n = 2;
        double2 y = new double2();
        double2 x = new double2();
        double2 Pb = new double2();

        LUDecomposition(matrix, out double2x2 L, out double2x2 U, out int2 P);

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

    public static REAL3 rotate(quaternion q, REAL3 v)
    {
#if USE_FLOAT
        return math.rotate(q, v);
#else
        return math.rotate(new float4x4(q, 0), v);
#endif
}
}

