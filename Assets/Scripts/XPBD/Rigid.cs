using UnityEngine;

namespace XPBD
{
    public class Rigid : Body
    {
        public enum eGeometryType { kSphere, kBox, kCylinder, kNone };
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
            Transform m_transform = this.transform;

            if (geometryType == eGeometryType.kBox)
            {
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
            }
            else if (geometryType == eGeometryType.kCylinder)
            {
                float radius = 0.5f * transform.localScale.x;
                float height = 2f * transform.localScale.y;
                ID = BackEnd.AddXPBDRigidCylinder(m_transform.position, m_transform.rotation, radius, height, mass);
                
            }
            Debug.Log(ID);
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            BackEnd.GetTransform(ID, out Vector3 pos, out Quaternion rot);

            transform.SetPositionAndRotation(pos, rot);
        }
    }
}

