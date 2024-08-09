using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

#if USE_FLOAT
using REAL = System.Single;
using REAL2 = Unity.Mathematics.float2;
using REAL3 = Unity.Mathematics.float3;
using REAL4 = Unity.Mathematics.float4;
using REAL2x2 = Unity.Mathematics.float2x2;
using REAL3x3 = Unity.Mathematics.float3x3;
using REAL3x4 = Unity.Mathematics.float3x4;
#else
using REAL = System.Double;
using REAL2 = Unity.Mathematics.double2;
using REAL3 = Unity.Mathematics.double3;
using REAL4 = Unity.Mathematics.double4;
using REAL2x2 = Unity.Mathematics.double2x2;
using REAL3x3 = Unity.Mathematics.double3x3;
using REAL3x4 = Unity.Mathematics.double3x4;
#endif

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
            
        }

        public virtual void Simulate(REAL dt) { }

        public virtual void UpdateVisual() { }

        public virtual void ApplyVelocity(REAL dt) { }
        public void UpdateCollisionMaterial()
        {
            collisions.Clear();
            collisionMaterial.CopyMatchingPropertiesFromMaterial(material);
        }
    }
}

