using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using XPBD;
using static UnityEditor.Searcher.SearcherWindow.Alignment;

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public class CustomPlane : MonoBehaviour
{
    private MeshRenderer meshRenderer;
    private MeshFilter meshFilter;
    Mesh mesh;

    [SerializeField, Min(1)] int numX = 100;
    [SerializeField, Min(1)] int numZ = 100;
    [SerializeField]int numCells;

    [SerializeField,Min(1)] int SizeX = 10;
    [SerializeField,Min(1)] int SizeZ = 10;
    void Start()
    {
        
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();

        numX = (int)math.floor(numX);
        numZ = (int)math.floor(numZ);

        numCells = (numX + 1) * (numZ + 1);
        //GenerateMesh();
       
    }

    // Update is called once per frame
    void Update()
    {
        Generate();
    }

    private void GenerateMesh()
    {
        //Generate the mesh's vertices and uvs
        Vector3[] positions = new Vector3[numCells];
        Vector2[] uvs = new Vector2[numCells];

        //Center of the mesh
        int cx = Mathf.FloorToInt(numX / 2f);
        int cz = Mathf.FloorToInt(numZ / 2f);

        for (int i = 0; i <= numX; i++)
        {
            for (int j = 0; j <= numZ; j++)
            {
                float posX = (i - cx);
                float posY = 0f;
                float posZ = (j - cz);

                positions[i * numZ + j] = (float3)new float3(posX, posY, posZ);

                float u = i / (float)numX;
                float v = j / (float)numZ;

                //uvs[i * numZ + j] = (float2)new float2(u, v);
            }
        }

        //Build triangles from the vertices
        //If the grid is 3x3 cells (16 vertices) we need a total of 18 triangles
        //-> 18*3 = 54 triangle indices are needed
        //numX is vertices: (4-3)*(4-3)*2*3 = 54
        int[] index = new int[(this.numX) * (this.numZ) * 2 * 3];

        int pos = 0;

        for (int i = 0; i < this.numX; i++)
        {
            for (int j = 0; j < this.numZ; j++)
            {
                int id0 = i * this.numZ + j;
                int id1 = i * this.numZ + j + 1;
                int id2 = (i + 1) * this.numZ + j + 1;
                int id3 = (i + 1) * this.numZ + j;

                index[pos++] = id0;
                index[pos++] = id1;
                index[pos++] = id2;

                index[pos++] = id0;
                index[pos++] = id2;
                index[pos++] = id3;
            }
        }

        //Generate the mesh itself
        Mesh newMesh = new()
        {
            name = "customPlane",
        };
        if (numCells > 65535)
            newMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        //To make it faster to update the mesh often
        newMesh.SetVertices(positions);
        newMesh.SetIndices(index, MeshTopology.Triangles, 0);
        newMesh.SetUVs(0, uvs);
        newMesh.MarkDynamic();

        mesh = newMesh;
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        mesh.RecalculateBounds();

        //mesh.bounds = new Bounds(mesh.bounds.size)
        //meshVertices = positions;

        meshFilter.mesh = mesh;
    }

    private void Generate()
    {
        GetComponent<MeshFilter>().mesh = mesh = new Mesh();
        mesh.name = "Procedural Grid";

        Vector3[] vertices = new Vector3[(numX + 1) * (numZ + 1)];
        Vector2[] uv = new Vector2[vertices.Length];

        float deltaX = (float)SizeX / numX;
        float deltaY = (float)SizeZ / numZ;
        for (int i = 0, y = 0; y <= numZ; y++)
        {
            for (int x = 0; x <= numZ; x++, i++)
            {
                vertices[i] = new Vector3(x * deltaX, 0, y * deltaY);

                uv[i] = new Vector2((float)x / numX, (float)y / numZ);
            }
        }
        mesh.vertices = vertices;

        int[] triangles = new int[numX * numZ * 6];
        for (int ti = 0, vi = 0, y = 0; y < numZ; y++, vi++)
        {
            for (int x = 0; x < numX; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + numX + 1;
                triangles[ti + 5] = vi + numX + 2;
            }
        }
        mesh.triangles = triangles;
        mesh.uv = uv;
    }
}
