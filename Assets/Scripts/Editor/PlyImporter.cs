using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor.AssetImporters;
using UnityEngine;
using UnityEngine.Rendering;

[ScriptedImporter(1, "ply")]
public class PlyImporter : ScriptedImporter
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

        #region Load Mesh
        NativeArray<Vector3> V;
        NativeArray<Vector3> N;
        NativeArray<int> F;
        NativeArray<Vector2> UV;
        int VSize, NSize, FSize, UVSize;

        unsafe
        {
            BackEnd.ReadPLY(ctx.assetPath, out var VPtr, out VSize, out var NPtr, out NSize,
                    out var FPtr, out FSize, out var UVPtr, out UVSize);
            V = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Vector3>(VPtr, VSize, Allocator.Temp);
            N = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Vector3>(NPtr, VSize, Allocator.Temp);
            F = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<int>(FPtr, 3 * FSize, Allocator.Temp);
            UV = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Vector2>(UVPtr, UVSize, Allocator.Temp);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref V, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref N, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref F, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref UV, AtomicSafetyHandle.Create());
#endif
            mesh.SetVertexBufferParams(VSize, BackEnd.VertexBufferLayout);
            mesh.SetIndexBufferParams(3 * FSize, IndexFormat.UInt32);

            mesh.SetVertices(V);
            mesh.SetIndices(F, MeshTopology.Triangles, 0);
            mesh.SetNormals(N);
            mesh.SetUVs(0, UV);

            mesh.MarkDynamic();
            mesh.MarkModified();
            mesh.RecalculateBounds();
        }
        #endregion
    }
}
