using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(Collider))]
    public class Primitive : MonoBehaviour
    {
        public new Collider collider { get; private set; }
        public Vector3 defaultNormal = Vector3.up;
        private bool isStarted = false;

        public Vector3 Position => transform.position;
        public Quaternion Rotation => transform.rotation;

        private void Awake()
        {
            gameObject.tag = "Primitive";
            collider = GetComponent<Collider>();
            Simulation.get.AddPrimitive(this);
        }

        private void Start()
        {
            isStarted = true;
        }

        private void OnDrawGizmos()
        {
            if (!isStarted)
                return;

            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(collider.bounds.center, collider.bounds.size);
        }
    }
}

