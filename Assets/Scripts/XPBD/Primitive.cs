using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(Collider))]
    public class Primitive : MonoBehaviour
    {
        public new Collider collider { get; private set; }
        public Geometry Geometry { get; private set; }
        //public Vector3 defaultNormal = Vector3.up;
        //private bool isStarted = false;

        public Vector3 Position => transform.position;
        public Quaternion Rotation => transform.rotation;

        private void Awake()
        {
            gameObject.tag = "Primitive";
            collider = GetComponent<Collider>();
            Geometry = GetComponent<Geometry>();
            Simulation.get.AddPrimitive(this);

            //Debug.Log(Geometry);
        }

        public bool IsInside(Vector3 point)
        {
            // Get local position
            Vector3 local = transform.InverseTransformPoint(point);
            //Debug.Log("Point:" + point);
            //Debug.Log("Local:" + local);
            return Geometry.IsInside(local);
        }

        public Vector3 ClosestSurfacePoint(Vector3 point, out Vector3 surfaceNormal)
        {
            // Get local position
            Vector3 local = transform.InverseTransformPoint(point);

            Vector3 result = Geometry.ClosestSurfacePoint(local, out surfaceNormal);

            result = transform.TransformPoint(result);
            surfaceNormal = transform.TransformDirection(surfaceNormal);

            //Debug.Log("Point:" + result);
            //Debug.Log("Normal:" + surfaceNormal);

            return result;
        }
    }
}

