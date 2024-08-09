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

namespace XPBD
{
    public abstract class Geometry : MonoBehaviour
    {
        public abstract REAL3 ClosestSurfacePoint(REAL3 point, out REAL3 surfaceNormal);
        public abstract bool IsInside(REAL3 point);
        public abstract bool Raycast(Ray ray, out RaycastHit hit, float maxDistance);
    }
}

