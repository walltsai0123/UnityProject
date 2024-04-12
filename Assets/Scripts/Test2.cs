using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

[ExecuteInEditMode()]
public class Test2 : MonoBehaviour
{
    public Color color = Color.black;
    public Vector4 vector;
    private void Update()
    {
        vector = color;
    }
}
