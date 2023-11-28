using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace XPBD
{
    [RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public unsafe class PhysicMesh : MonoBehaviour
    {
        private MeshFilter meshFilter;
        public Mesh mesh { get; private set; }
        public MeshRenderer meshRenderer { get; private set; }
        
        public TetMeshData DataRowMajor { get; private set; }
        public TetMeshState* state;

        public int[] tets;
        public void Initialize()
        {
            meshFilter = GetComponent<MeshFilter>();
            mesh = meshFilter.mesh;
            meshRenderer = GetComponent<MeshRenderer>();

            //DataRowMajor = new TetMeshData(this);
            //state = BackEnd.InitTetMeshState(DataRowMajor.GetNative());

            Debug.Log("PhysicMesh Initialize");
        }
        public void UpdateMesh(Vector3[] pos)
        {
            //DataRowMajor.ApplyDirty(state);
            //DataRowMajor.ApplyDirtyToMesh(mesh);


            mesh.SetVertices(pos);
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
        }
        public void UpdateMesh(NativeArray<float3> pos)
        {
            mesh.SetVertices<float3>(pos);
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
        }


        public void Show(bool showTets)
        {
            meshRenderer.enabled = showTets;
        }

        private void OnDestroy()
        {
            Dispose();
            Debug.Log("PhysicMesh Destroy");
        }
        private void Dispose()
        {
            if (DataRowMajor != null)
            {
                DataRowMajor.Dispose();
            }
            BackEnd.DisposeTetMeshState(state);
        }
    }
}
