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

public interface IGrabbable
{
    public bool Grabbable { get; }
    public void StartGrab(REAL3 grabPos);

    public void MoveGrabbed(REAL3 grabPos);

    public void EndGrab(REAL3 grabPos, REAL3 vel);

    public void IsRayHittingBody(Ray ray, out CustomHit hit);

    public REAL3 GetGrabbedPos(); 
}
