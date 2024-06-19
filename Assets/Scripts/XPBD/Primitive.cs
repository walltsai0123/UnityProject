using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    [RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public class Primitive : MonoBehaviour
    {
        public Geometry Geometry { get; protected set; }

        public Mesh mesh { get; protected set; }
        public Material material { get; protected set; }
        public Material collisionMaterial { get; private set; }
        public Vector3 Position => transform.position;
        public Quaternion Rotation => transform.rotation;

        public List<MyCollision> collisions { get; protected set; }

        protected virtual void Awake()
        {
            gameObject.tag = "Primitive";
            Geometry = GetComponent<Geometry>();
            mesh = GetComponent<MeshFilter>().mesh;
            material = GetComponent<MeshRenderer>().material;
        }

        protected virtual void Start()
        {
            collisions = new List<MyCollision>();
            collisionMaterial = new(Simulation.get.textureFrictionShader);
            Simulation.get.primitives.Add(this);
        }

        private void OnDrawGizmos()
        {
            if (collisions == null)
            {
                return;
            }
            foreach (MyCollision collision in collisions)
            {
                DrawArrow.ForGizmo(collision.q, collision.T * collision.frictionCoef[0], Color.red);
                DrawArrow.ForGizmo(collision.q, collision.B * collision.frictionCoef[1], Color.green);
                DrawArrow.ForGizmo(collision.q, -collision.T * collision.frictionCoef[2], Color.blue);
                DrawArrow.ForGizmo(collision.q, -collision.B * collision.frictionCoef[3], Color.black);


                //Debug.Log(collision.frictionCoef);
                //Debug.Log(collision.fn);
                //Debug.Log(collision.ft);
                //Debug.Log(collision.fb);
                //Debug.Log(collision.T);
                //Debug.Log(collision.B);

            }
        }

        public virtual void Simulate(float dt) { }

        public virtual void UpdateVisual() { }

        public virtual void ApplyVelocity(float dt) { }
        public void UpdateCollisionMaterial()
        {
            collisions.Clear();
            collisionMaterial.CopyMatchingPropertiesFromMaterial(material);
        }
    }
}

