using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TerrainTest : MonoBehaviour
{
    [SerializeField] RenderTexture renderTexture;
    [SerializeField] Terrain terrain;
    [SerializeField] Material material;


    RenderTexture heightMap;
    // Update is called once per frame
    private void Start()
    {
        TerrainData terrainData = terrain.terrainData;

        int heightmapWidth = terrainData.heightmapResolution;
        int heightmapHeight = terrainData.heightmapResolution;

        float x = 50 / terrainData.size.x;
        float y = 50 / terrainData.size.z;
        float height = terrainData.GetInterpolatedHeight(x, y);
        Debug.Log(heightmapWidth + " " + heightmapHeight);
        Debug.Log(terrainData.heightmapScale);
        Debug.Log(terrainData.size);
        Debug.Log(height);
        Debug.Log(terrain.terrainData.bounds.max);
        renderTexture = terrainData.heightmapTexture;
        Debug.Log(renderTexture.enableRandomWrite);

        Texture2D tex = new Texture2D(renderTexture.width, renderTexture.width, TextureFormat.R16, false);
        RenderTexture.active = renderTexture;
        tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        tex.Apply();

        Debug.Log(tex.GetPixel(Mathf.RoundToInt(x / tex.width), Mathf.RoundToInt(y / tex.height)));

        material.SetTexture("_MainTex", renderTexture);
    }
    void Update()
    {
    }
}
