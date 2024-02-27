using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test2 : MonoBehaviour
{
    public new Collider collider;
    private bool isStarted = false;

    private void Start()
    {
        isStarted = true;
        collider = GetComponent<Collider>();
    }

    private void OnDrawGizmos()
    {
        if (!isStarted)
            return;

        Ray ray = new(transform.position, Vector3.up);

        bool collide = collider.Raycast(ray, out RaycastHit info, float.MaxValue);

        Debug.Log(collide);

        Gizmos.DrawSphere(info.point, 0.1f);
        DrawArrow.ForGizmo(transform.position, Vector3.up * info.distance);
    }
}
