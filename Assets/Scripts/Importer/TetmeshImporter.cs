using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor.AssetImporters;
using UnityEngine;

[ScriptedImporter(1, "tetmesh")]
public class TetmeshImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext ctx)
    {
        #region Create Imported GameObject
        var gameObject = new GameObject();
        ctx.AddObjectToAsset("Main Object", gameObject);
        ctx.SetMainObject(gameObject);

        var mesh = new Mesh();
        var startIndex = ctx.assetPath.LastIndexOf("/") + 1;
        var length = ctx.assetPath.LastIndexOf(".") - startIndex;
        var meshName = (startIndex >= 0 && length > 0) ? ctx.assetPath.Substring(startIndex, length) : "imported-mesh";
        mesh.name = meshName;
        ctx.AddObjectToAsset("Mesh", mesh);

        var meshFilter = gameObject.AddComponent<MeshFilter>();
        meshFilter.mesh = mesh;

        var meshRenderer = gameObject.AddComponent<MeshRenderer>();
        var newMaterial = new Material(Shader.Find("Standard"));
        ctx.AddObjectToAsset("Material", newMaterial);
        meshRenderer.material = newMaterial;

        #endregion

        #region Load Mesh File
        try
        {
            using StreamReader sr = new StreamReader(ctx.assetPath);
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                Debug.Log(line + " " + line.Length);
            }
        }
        catch (Exception e)
        {
            Debug.Log("The file could not be read:" + e.Message);
        }
        #endregion
    }
}
