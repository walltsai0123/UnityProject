using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public class Test2 : MonoBehaviour
{
    public TetrahedronMesh tetmesh;

    private MeshFilter meshFilter;
    public Mesh mesh { get; private set; }
    public MeshRenderer meshRenderer { get; private set; }

    private void Start()
    {
        if (tetmesh == null)
            return;

        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Standard"));

        mesh.SetVertices(tetmesh.vertices);
        mesh.SetIndices(tetmesh.faces, MeshTopology.Triangles, 0);
        mesh.MarkDynamic();
        mesh.MarkModified();
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();

        Debug.Log(mesh.normals.Length);
    }
}
