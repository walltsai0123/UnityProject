using UnityEngine;

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public unsafe class VisMesh : MonoBehaviour
{
    private MeshFilter meshFilter;
    public Mesh mesh { get; private set; }
    public MeshRenderer meshRenderer { get; private set; }

    public VisMeshData DataRowMajor { get; private set; }
    public MeshState* state;

    public void Initialized()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        meshRenderer = GetComponent<MeshRenderer>();

        DataRowMajor = new VisMeshData(this);
        state = BackEnd.InitMeshState(DataRowMajor.GetNative());

        Debug.Log("VisMesh Initialize");
    }

    public void UpdateMesh()
    {
        DataRowMajor.ApplyDirty(state);
        DataRowMajor.ApplyDirtyToMesh(mesh);
    }

    public void Show(bool show)
    {
        meshRenderer.enabled = show;
    }

    private void OnDestroy()
    {
        Dispose();
        Debug.Log("VisMesh Destroy");
    }
    private void Dispose()
    {
        if (DataRowMajor != null)
        {
            DataRowMajor.Dispose();
        }
        BackEnd.DisposeMeshState(state);
    }
}
