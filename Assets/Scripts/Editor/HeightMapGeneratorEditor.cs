using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(HeightMapGenerator))]
public class HeightMapGeneratorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        HeightMapGenerator mapGen = (HeightMapGenerator)target;

        if (DrawDefaultInspector())
        {
            if (mapGen.autoUpdate)
            {
                mapGen.GenerateHeightMap();
                mapGen.ConstructMesh();
            }
        }

        if (GUILayout.Button("Generate"))
        {
            mapGen.GenerateHeightMap();
            mapGen.ConstructMesh();
        }

        if(GUILayout.Button("Save Map"))
        {
            mapGen.SaveMap();
        }
    }
}
