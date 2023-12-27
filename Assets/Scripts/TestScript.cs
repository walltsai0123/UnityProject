using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class TestScript : MonoBehaviour
{
    void Start()
    {
        float4 newfloat4 = new(3,6,4,2);
        for (int i = 0; i < 4; ++i)
            Debug.Log(newfloat4[i]);

        Debug.Log(newfloat4.x);
        Debug.Log(newfloat4.y);
        Debug.Log(newfloat4.z);
        Debug.Log(newfloat4.w);
    }

}
