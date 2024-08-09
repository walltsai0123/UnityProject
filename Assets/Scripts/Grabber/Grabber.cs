using System.Collections.Generic;
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
public class Grabber
{
    //Data needed 
    private readonly Camera mainCamera;

    //The mesh we grab
    private IGrabbable grabbedBody = null;

    //Mesh grabbing data

    //When we have grabbed a mesh by using ray-triangle itersection we identify the closest vertex. The distance from camera to this vertex is constant so we can move it around without doing another ray-triangle itersection  
    private REAL distanceToGrabPos;
    //The value of sin(Angle), Angle is the angle between camera forward REAL and ray
    private REAL rayAngle;

    //To give the mesh a velocity when we release it
    private REAL3 lastGrabPos;

    public Grabber(Camera mainCamera)
    {
        this.mainCamera = mainCamera;
    }

    public void StartGrab(List<IGrabbable> bodies)
    {
        if (grabbedBody != null)
        {
            return;
        }

        //A ray from the mouse into the scene
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);

        REAL maxDist = REAL.MaxValue;

        IGrabbable closestBody = null;

        CustomHit closestHit = default;

        foreach (IGrabbable body in bodies)
        {
            if (!body.Grabbable)
                continue;

            body.IsRayHittingBody(ray, out CustomHit hit);

            if (hit != null)
            {
                if (hit.distance < maxDist)
                {
                    closestBody = body;

                    maxDist = hit.distance;

                    closestHit = hit;
                }
            }
        }
        if (closestBody != null)
        {
            grabbedBody = closestBody;

            //StartGrab is finding the closest vertex and setting it to the position where the ray hit the triangle
            closestBody.StartGrab(closestHit.location);

            lastGrabPos = closestHit.location;

            rayAngle = Mathf.Abs(math.dot(ray.direction, mainCamera.transform.forward));

            //distanceToGrabPos = (ray.origin - hit.location).magnitude;
            distanceToGrabPos = closestHit.distance * rayAngle;
        }
    }

    public void MoveGrab()
    {
        if (grabbedBody == null)
        {
            return;
        }

        //A ray from the mouse into the scene
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        REAL3 rayO = (float3)ray.origin;
        REAL3 rayDir = (float3)ray.direction;

        rayAngle = Mathf.Abs(math.dot(ray.direction, mainCamera.transform.forward));

        REAL3 vertexPos = rayO + rayDir * distanceToGrabPos / rayAngle;

        //Cache the old pos before we assign it
        lastGrabPos = grabbedBody.GetGrabbedPos();

        //Moved the vertex to the new pos
        grabbedBody.MoveGrabbed(vertexPos);
    }

    public void EndGrab()
    {
        if (grabbedBody == null)
        {
            return;
        }

        //Add a velocity to the ball

        //A ray from the mouse into the scene
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        REAL3 rayO = (float3)ray.origin;
        REAL3 rayDir = (float3)ray.direction;

        REAL3 grabPos = rayO + rayDir * distanceToGrabPos / rayAngle;

        REAL vel = math.length(grabPos - lastGrabPos) / Time.deltaTime;

        REAL3 dir = math.normalize(grabPos - lastGrabPos);

        grabbedBody.EndGrab(grabPos, dir * vel);

        grabbedBody = null;
    }
}
