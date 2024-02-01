using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using XPBD;

public class TestScript : MonoBehaviour
{
    [SerializeField] SoftBody softBody;
    public float accel;
    public float3 dir;
    private bool started = false;

    private void Start()
    {
        started = true;

        Color color = Color.yellow;
        Vector4 v = color;

        Debug.Log(color);
        Debug.Log(v.x);
        Debug.Log(v.y);
        Debug.Log(v.z);
        Debug.Log(v.w);
    }
    private void Update()
    {
        float3 final = float3.zero;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            final += new float3(0, 0, 1);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            final += new float3(0, 0, -1);
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            final += new float3(-1, 0, 0);
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            final += new float3(1, 0, 0);
        }

        dir = final;

        softBody.fext = final * accel;
    }

    private void OnDrawGizmos()
    {
        if (!started)
            return;
        DrawArrow.ForGizmo(softBody.Pos[0], dir * accel, Color.red);
    }
}
