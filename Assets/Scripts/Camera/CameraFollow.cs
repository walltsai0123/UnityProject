using UnityEngine;
using XPBD;
using Unity.Mathematics;

[RequireComponent(typeof(Camera))]
public class CameraFollow : MonoBehaviour
{
    public SoftBody softBody;
    public float dist = 10f;
    // Update is called once per frame
    void Update()
    {
        if(softBody == null)
            return;

        transform.position = (float3)softBody.Pos[0];

        transform.position += dist * Vector3.back + dist * Vector3.up;

        transform.LookAt((float3)softBody.Pos[0]);
    }
}
