using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

[ExecuteInEditMode()]
public class Test2 : MonoBehaviour
{
    public Quaternion q1 = Quaternion.identity;
    public Quaternion q2 = Quaternion.identity;
    public Quaternion result = Quaternion.identity;
    [Header("Angle axis")]
    public Vector3 axis;
    public float angle;

    [Header("Rotate vector")]
    public Vector3 vA;
    public Vector3 vB;
    private void Update()
    {
        result = q1 * Quaternion.Inverse(q2);
        result.ToAngleAxis(out angle, out axis);

        vB = Quaternion.Inverse(q2) * vA;
        vB = math.rotate(math.conjugate(q2), vA);
    }
}
