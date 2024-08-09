#ifndef _UTIL_H_
#define _UTIL_H_

#include<./settings.hlsl>

void LUDecomposition(REAL2x2 A, out REAL2x2 L, out REAL2x2 U, out int2 P)
{
    int n = 2;
    L = 0;
    U = 0;
    P = 0;

    for (int i = 0; i < n; i++)
        P[i] = i;

    // Partial Pivoting
    if (abs(A[1][0]) > abs(A[0][0]))
    {
        // Swap rows in permutation array
        int temp = P[0];
        P[0] = P[1];
        P[1] = temp;

        // Swap rows in A
        for (int i = 0; i < n; i++)
        {
            REAL tempA = A[0][i];
            A[0][i] = A[1][i];
            A[1][i] = tempA;
        }
    }

    // U matrix is upper triangular
    U[0][0] = A[0][0];
    U[0][1] = A[1][0];
    U[1][0] = 0;
    U[1][1] = A[1][1] - (A[1][0] * A[0][1] / A[0][0]);

    // L matrix is lower triangular
    L[0][0] = 1;
    L[0][1] = 0;
    L[1][0] = A[0][1] / A[0][0];
    L[1][1] = 1;
}

REAL2 LUSolve(REAL2x2 A, REAL2 b)
{
    // Extract matrix elements
    REAL a11 = A[0][0];
    REAL a12 = A[0][1];
    REAL a21 = A[1][0];
    REAL a22 = A[1][1];

    // Partial pivoting
    bool pivot = abs(a21) > abs(a11);
    if (pivot) {
        // Swap rows
        REAL tmp = a11;
        a11 = a21;
        a21 = tmp;

        tmp = a12;
        a12 = a22;
        a22 = tmp;

        tmp = b.x;
        b.x = b.y;
        b.y = tmp;
    }

    // LU decomposition
    REAL L21 = a21 / a11;
    REAL U11 = a11;
    REAL U12 = a12;
    REAL U22 = a22 - L21 * a12;

    // Forward substitution to solve Ly = b
    REAL y1 = b.x;
    REAL y2 = b.y - L21 * b.x;

    // Backward substitution to solve Ux = y
    REAL2 x;
    x.y = y2 / U22;
    x.x = (y1 - U12 * x.y) / U11;

    return x;
}

REAL3 OrthogonalVector(REAL3 v)
{
    if(dot(v, v) < EPSILON_SQUARE)
        return 0;

    REAL3 A = normalize(v);
    REAL3 B = cross(A, REAL3(1,0,0));

    if (dot(B, B) < EPSILON_SQUARE)
    {
        B = cross(A, REAL3(0,0,-1));
    }
    B = normalize(B);

    return B;
}
#endif