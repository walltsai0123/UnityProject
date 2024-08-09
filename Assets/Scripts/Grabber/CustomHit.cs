using System.Collections;
using System.Collections.Generic;
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
public class CustomHit
{
    //The distance along the ray to where the ray hit the object
    public REAL distance;

    //Point of intersection
    public REAL3 location;
    //Normal of the surface where the ray hit
    public REAL3 normal;

    //What is index? Currently assumed to be the index of the first vertex of the triangle
    public int index;

    public CustomHit(REAL distance, REAL3 location, REAL3 normal, int index = -1)
    {
        this.distance = distance;
        this.location = location;
        this.normal = normal;
        this.index = index;
    }
}
