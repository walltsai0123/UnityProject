using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using XPBD;
using System;

public class TestScript : MonoBehaviour
{
    [SerializeField] SoftBody softBody;
    public float accel;
    public float3 dir;
    private bool started = false;
    public Color color = Color.white;

    private void Start()
    {
        started = true;

        float2x2 A = new float2x2(1266.899f, 2194.333f, 2194.333f, 3800.695f);

        float2 b = new float2( -1256f, 0f );
        float2 x = Util.LUSolve(A, b);

        float3x3 T = new float3x3(1, 2, 3, 4, 5, 6, 7, 8, 9);

        float3 T1 = new float3(1, 2, 3);
        Debug.Log(math.length(T1).ToString("G"));
        Debug.Log(math.lengthsq(T1).ToString("G"));
        Debug.Log(math.dot(T1, T1).ToString("G"));

        //Debug.Log("Solution x: ");
        //Debug.Log(x[0]);
        //Debug.Log(x[1]);
    }
    private void Update()
    {
        if (softBody == null)
            return;


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

        dir = math.normalizesafe(final, float3.zero);

        softBody.fext = final * accel;
    }

    private void OnDrawGizmos()
    {
        if (!started || softBody == null)
            return;
        DrawArrow.ForGizmo(softBody.Pos[0], dir * accel, color);
    }

}
