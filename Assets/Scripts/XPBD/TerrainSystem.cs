using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using XPBD;

public class TerrainSystem
{
    public List<MyTerrain> Terrains => myTerrains;
    List<MyTerrain> myTerrains = new();
    Dictionary<int2, MyTerrain> terrainMap = new();

    float terrainSize;

    public void TerrainDeformation(SoftBodySystem sbs, float dt)
    {
        foreach (MyTerrain terrain in myTerrains)
            terrain.PaintFootPrints(sbs, dt);
    }
    public void AddTerrain(MyTerrain terrain)
    {
        if (terrain == null) return;

        // Check if terrain size is equal
        if(myTerrains.Count > 0)
        {
            if(terrainSize != terrain.Terrain.terrainData.size.x)
            {
                Debug.LogError($"Terrain size is different {terrainSize}, {terrain.Terrain.terrainData.size.x}");
                EditorApplication.isPaused = true;
            }
        }
        else
        {
            terrainSize = terrain.Terrain.terrainData.size.x;
        }
        float3 terrainPos = terrain.transform.position;
        float2 key = math.floor(terrainPos.xz / terrainSize);
        
        myTerrains.Add(terrain);
        terrainMap.Add((int2)key, terrain);
    }
    public MyTerrain GetTerrain(float3 pos)
    {
        float2 posHash = math.floor(pos.xz / terrainSize);
        int2 key = (int2)posHash;
        if (terrainMap.TryGetValue(key, out MyTerrain theTerrain))
        { 
            return theTerrain;
        }
        return null;
    }
}