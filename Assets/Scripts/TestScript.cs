using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class TestScript : MonoBehaviour
{
    private List<ContactPoint> contactPoints;
    

    public void Awake()
    {
        contactPoints = new();
    }

    private void OnCollisionEnter(Collision collision)
    {
        collision.GetContacts(contactPoints);
    }

    public void OnDrawGizmos()
    {
        if (contactPoints == null)
            return;
        foreach(var cp in contactPoints)
        {
            Vector3 point = cp.point;
            Vector3 normal = cp.normal;

            Gizmos.DrawSphere(point, 0.05f);
            Gizmos.DrawRay(point, normal);
        }
    }
}
