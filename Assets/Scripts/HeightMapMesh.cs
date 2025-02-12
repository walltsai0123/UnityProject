using UnityEngine;

[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
public class HeightMapMesh : MonoBehaviour
{
    protected MeshRenderer meshRenderer;
    protected MeshFilter meshFilter;
    protected Mesh mesh;
    public Material material;

    private float SizeX = 1000;
    private float SizeZ = 1000;
    //int numX, numZ;
    readonly int meshX = 32;
    readonly int meshZ = 32;

    public RenderTexture Height_Map => height_map;
    public RenderTexture Normal_Map => normal_map;

    [SerializeField] protected RenderTexture height_map;
    [SerializeField] protected RenderTexture normal_map;

    public virtual void Initialize(int width, float Size)
    {
        meshFilter = GetComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();
        //material = meshRenderer.material;
        meshRenderer.sharedMaterial = material;
        height_map = new RenderTexture(width, width, 0, RenderTextureFormat.ARGB32);
        normal_map = new RenderTexture(width - 1, width - 1, 0, RenderTextureFormat.ARGB32);

        height_map.name = gameObject.name + " height";
        normal_map.name = gameObject.name + " normal";

        height_map.enableRandomWrite = true;

        SizeX = SizeZ = Size;

        GeneratePlaneMesh();

        SetMaterialProperties();
    }
    public void SetMaxHeight(float maxHeight)
    {
        material.SetFloat("_Parallax", (float)maxHeight);
    }
    protected virtual void SetMaterialProperties()
    {
        material.SetTexture("_ParallaxMap", height_map);
        if (normal_map)
            material.SetTexture("_TerrainNormalMap", normal_map);
        material.SetFloat("_Parallax", (float)200);
    }

    private void GeneratePlaneMesh()
    {
        if(meshFilter.sharedMesh && mesh)
        {
            mesh.Clear();
        }
        else
        {
            mesh = new()
            {
                name = gameObject.name + " Procedural Grid"
            };
        }

        meshFilter.sharedMesh = mesh;

        Vector3[] vertices = new Vector3[(meshX + 1) * (meshZ + 1)];
        Vector2[] uv = new Vector2[vertices.Length];

        float deltaX = (float)SizeX / meshX;
        float deltaY = (float)SizeZ / meshZ;
        for (int i = 0, y = 0; y <= meshZ; y++)
        {
            for (int x = 0; x <= meshZ; x++, i++)
            {
                vertices[i] = new Vector3(x * deltaX, 0, y * deltaY);

                uv[i] = new Vector2((float)x / meshX, (float)y / meshZ);
            }
        }
        mesh.vertices = vertices;

        int[] triangles = new int[meshX * meshZ * 6];
        for (int ti = 0, vi = 0, y = 0; y < meshZ; y++, vi++)
        {
            for (int x = 0; x < meshX; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + meshX + 1;
                triangles[ti + 5] = vi + meshX + 2;
            }
        }
        mesh.triangles = triangles;
        mesh.uv = uv;
    }


}
