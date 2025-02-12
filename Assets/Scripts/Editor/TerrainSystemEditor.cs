using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using XPBD;

[CustomEditor(typeof(TerrainSystem))]
public class TerrainSystemEditor : Editor
{
    public override void OnInspectorGUI()
    {
        TerrainSystem terrainSystem = (TerrainSystem)target;

        if (DrawDefaultInspector())
        {
            //terrainSystem.ConstructMesh();
        }

    }
}
