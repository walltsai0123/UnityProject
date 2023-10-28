using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public class Rigid : Body
    {
        public Rigidbody m_rigidbody;
        public Vector3 inertia;

        public float mass;

        private void Awake()
        {
            m_rigidbody = GetComponent<Rigidbody>();
            inertia = m_rigidbody.inertiaTensor;
            m_rigidbody.isKinematic = true;

            ID = BackEnd.AddXPBDRigidBody(transform.position, transform.rotation, inertia, mass);
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            BackEnd.GetTransform(ID, out Vector3 pos, out Quaternion rot);

            transform.SetPositionAndRotation(pos, rot);
        }
    }
}

