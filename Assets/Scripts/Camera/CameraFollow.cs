using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using XPBD;

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

        transform.position = softBody.Pos[0];

        transform.position += dist * Vector3.back + dist * Vector3.up;

        transform.LookAt(softBody.Pos[0]);
    }
}
