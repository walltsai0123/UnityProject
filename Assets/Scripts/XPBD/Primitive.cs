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

        private void Awake()
        {
            gameObject.tag = "Primitive";
            collider = GetComponent<Collider>();
            Simulation.get.AddPrimitive(this);
        }
    }
}

