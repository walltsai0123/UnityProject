using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

[ExecuteInEditMode]
public class Test2 : MonoBehaviour
{
    public Vector3 v0; 
    [DebugOnly] public Vector3 v1;
    [DebugOnly] public Vector3 v2;
    public Quaternion q1 = Quaternion.identity;
    public Quaternion q2 = Quaternion.identity;
    private void Update()
    {
        v1 = math.rotate(q1, v0);
        v1 = math.rotate(q2, v1);

        var q = math.mul(q2, q1);
        v2 = math.mul(q, v0);
    }
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, v0);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, v1);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, v2);
    }
}
