using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public unsafe class SoftBody : Body
    {
        public VisMesh visMesh;
        public PhysicMesh physicMesh;

        public float mass = 1f, mu = 10f, lambda = 1000f;
        public bool showTet = false;

        private void Awake()
        {
            visMesh.Initialized();
            physicMesh.Initialize();
            ID = BackEnd.AddXPBDSoftBody(visMesh.state, physicMesh.state, transform.position, transform.rotation, mass, mu, lambda);
            Debug.Log(ID);
        }
        private void FixedUpdate()
        {
            physicMesh.Show(showTet);

            physicMesh.UpdateMesh();
            visMesh.UpdateMesh();

            mu = Mathf.Max(0.001f, mu);
            lambda = Mathf.Max(0.001f, lambda);
            BackEnd.setBodyMaterial(ID, mu, lambda);

            // BackEnd.GetTransform(ID, out Vector3 pos, out Quaternion rot);
            // transform.SetPositionAndRotation(pos, rot);
        }
    }
}

