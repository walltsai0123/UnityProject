using UnityEngine;

namespace XPBD
{
    public class Rigid : Body
    {
        public enum eGeometryType { kSphere, kBox, kNone };
        public Rigidbody m_rigidbody;
        public Vector3 inertia;

        public float mass;
        public eGeometryType geometryType = eGeometryType.kNone;

        private void Awake()
        {
            if(geometryType == eGeometryType.kNone)
            {
                Debug.Log("geometry unset");
                return;
            }

            if (geometryType == eGeometryType.kBox)
            {
                Transform m_transform = this.transform;
                BoxCollider box = GetComponent<BoxCollider>();
                if(box == null)
                {
                    Debug.Log("BoxCollider not found");
                    return;
                }
                Vector3 boxSize = box.size;
                boxSize.x *= transform.localScale.x;
                boxSize.y *= transform.localScale.y;
                boxSize.z *= transform.localScale.z;
                ID = BackEnd.AddXPBDRigidBox(m_transform.position, m_transform.rotation, boxSize, mass);
                Debug.Log(ID + boxSize.ToString());
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            BackEnd.GetTransform(ID, out Vector3 pos, out Quaternion rot);

            transform.SetPositionAndRotation(pos, rot);
        }
    }
}

