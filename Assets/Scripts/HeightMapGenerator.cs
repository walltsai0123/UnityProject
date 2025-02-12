using System.IO;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Experimental.Rendering;

public class HeightMapGenerator : MonoBehaviour
{
    [Header("Map Settings")]
    public int seed;
    public bool randomizeSeed;

    [SerializeField, Min(1)] int mapSize = 128;
    [SerializeField, Min(1)] int numOctaves = 7;
    [Range(0,1)]public float persistence = .5f;
    public float lacunarity = 2;
    public float initialScale = 2;

    public Texture2D height_map;
    float[] map;
    Mesh mesh;

    MeshRenderer meshRenderer;
    MeshFilter meshFilter;

    [Header("Mesh Settings")]
    public bool autoUpdate = false;
    public float scale = 20;
    public float elevationScale = 10;
    public Material material;
    public bool color32 = true;

    public void GenerateHeightMap()
    {
        var map = new float[mapSize * mapSize];
        seed = (randomizeSeed) ? UnityEngine.Random.Range(-10000, 10000) : seed;
        var prng = new System.Random(seed);

        Vector2[] offsets = new Vector2[numOctaves];
        for (int i = 0; i < numOctaves; i++)
        {
            offsets[i] = new Vector2(prng.Next(-1000, 1000), prng.Next(-1000, 1000));
        }

        float minValue = float.MaxValue;
        float maxValue = float.MinValue;

        for (int y = 0; y < mapSize; y++)
        {
            for (int x = 0; x < mapSize; x++)
            {
                float noiseValue = 0;
                float scale = initialScale;
                float weight = 1;
                float size = mapSize - 1;
                for (int i = 0; i < numOctaves; i++)
                {
                    Vector2 p = offsets[i] + new Vector2(x / size, y / size) * scale;
                    //noiseValue += Mathf.PerlinNoise(p.x, p.y) * weight;
                    noiseValue += noise.snoise(new float2(p.x, p.y)) * weight;
                    weight *= persistence;
                    scale *= lacunarity;
                }
                map[y * mapSize + x] = noiseValue;
                minValue = Mathf.Min(noiseValue, minValue);
                maxValue = Mathf.Max(noiseValue, maxValue);
            }
        }

        // Normalize
        if (maxValue != minValue)
        {
            for (int i = 0; i < map.Length; i++)
            {
                map[i] = (map[i] - minValue) / (maxValue - minValue);
            }
        }
        this.map = map;


        height_map = new Texture2D(mapSize, mapSize);

        Color[] colors = new Color[mapSize * mapSize];
        Color32[] color32Map = new Color32[mapSize * mapSize];
        for (int y = 0; y < mapSize; y++)
        {
            for (int x = 0; x < mapSize; x++)
            {
                colors[y * mapSize + x] = Color.Lerp(Color.black, Color.white, map[y * mapSize + x]);
                color32Map[y * mapSize + x] = Color32.Lerp(Color.black, Color.white, map[y * mapSize + x]);
            }
        }
        if (color32)
            height_map.SetPixels32(color32Map);
        else
            height_map.SetPixels(colors);

        height_map.Apply();

    }

    public void ConstructMesh()
    {
        int meshSize = 10;
        Vector3[] verts = new Vector3[meshSize * meshSize];
        Vector2[] uv = new Vector2[verts.Length];
        int[] triangles = new int[(meshSize - 1) * (meshSize - 1) * 6];
        int t = 0;

        for (int i = 0; i < meshSize * meshSize; i++)
        {
            int x = i % meshSize;
            int y = i / meshSize;
            //int borderedMapIndex = (y + erosionBrushRadius) * mapSizeWithBorder + x + erosionBrushRadius;
            int meshMapIndex = y * meshSize + x;

            Vector2 percent = new Vector2(x / (meshSize - 1f), y / (meshSize - 1f));
            Vector3 pos = new Vector3(percent.x, 0, percent.y) * scale;

           // float normalizedHeight = map[borderedMapIndex];
           // pos += Vector3.up * normalizedHeight * elevationScale;
            verts[meshMapIndex] = pos;
            uv[i] = new Vector2((float)x / (meshSize - 1f), (float)y / (meshSize - 1f));

            // Construct triangles
            if (x != meshSize - 1 && y != meshSize - 1)
            {
                t = (y * (meshSize - 1) + x) * 3 * 2;

                triangles[t + 0] = meshMapIndex + meshSize;
                triangles[t + 1] = meshMapIndex + meshSize + 1;
                triangles[t + 2] = meshMapIndex;

                triangles[t + 3] = meshMapIndex + meshSize + 1;
                triangles[t + 4] = meshMapIndex + 1;
                triangles[t + 5] = meshMapIndex;
                t += 6;
            }
        }

        if (mesh == null)
        {
            mesh = new Mesh();
        }
        else
        {
            mesh.Clear();
        }
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.vertices = verts;
        mesh.triangles = triangles;
        mesh.uv = uv;
        mesh.RecalculateNormals();

        Transform meshHolder = this.transform;
        if (!meshHolder.gameObject.GetComponent<MeshFilter>())
        {
            meshHolder.gameObject.AddComponent<MeshFilter>();
        }
        if (!meshHolder.GetComponent<MeshRenderer>())
        {
            meshHolder.gameObject.AddComponent<MeshRenderer>();
        }

        meshRenderer = meshHolder.GetComponent<MeshRenderer>();
        meshFilter = meshHolder.GetComponent<MeshFilter>();

        if (material == null)
            material = new Material(Shader.Find("Custom/Tessallation"));

        meshFilter.sharedMesh = mesh;
        meshRenderer.sharedMaterial = material;

        material.SetTexture("_MainTex", height_map);
        material.SetTexture("_ParallaxMap", height_map);
        material.SetFloat("_Parallax", elevationScale);
        material.SetTexture("_TerrainNormalMap", null);
    }

    public void SaveMap()
    {
        if(height_map == null)
        {
            Debug.LogWarning("Map is not created");
            return;
        }
        byte[] mapBytes = ImageConversion.EncodeToPNG(height_map);
        //Object.Destroy(normalMap);

        string fileName = System.DateTime.Now.ToString("MMddyyyy_hh_mm_ss");
        File.WriteAllBytes(Application.dataPath + "/./Textures/Generate Map/height " + fileName + ".png", mapBytes);
    }
}
