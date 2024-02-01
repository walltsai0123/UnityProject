using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace XPBD
{
    [RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public class VisMesh : MonoBehaviour
    {
        private MeshFilter meshFilter;
        public Mesh mesh { get; private set; }
        public MeshRenderer meshRenderer { get; private set; }

        public void Initialize()
        {
            meshFilter = GetComponent<MeshFilter>();
            mesh = meshFilter.mesh;
            meshRenderer = GetComponent<MeshRenderer>();

            Debug.Log("VisMesh Initialize");
        }
        public void UpdateMesh(NativeArray<float3> pos)
        {
            mesh.SetVertices<float3>(pos);
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
        }

        public void Show(bool show)
        {
            meshRenderer.enabled = show;
        }

    }
}

