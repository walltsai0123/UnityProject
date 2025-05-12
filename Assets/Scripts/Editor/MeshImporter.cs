using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor.AssetImporters;
using UnityEngine;
using UnityEngine.Rendering;

[ScriptedImporter(1, "mmesh")]
public class MeshImporter : ScriptedImporter
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

        //var tetMesh = gameObject.AddComponent<TetMesh>();
        //tetMesh.tetFileName = ctx.assetPath;

        //var physicMesh = gameObject.AddComponent<XPBD.PhysicMesh>();
        
        #endregion

        #region Load Mesh
        NativeArray<Vector3> V;
        NativeArray<Vector3> N;
        NativeArray<int> F;
        NativeArray<int> T;
        int VSize, NSize, FSize, TSize;

        unsafe
        {
            BackEnd.ReadMESH(ctx.assetPath, out var VPtr, out VSize, out var NPtr, out NSize,
                    out var FPtr, out FSize, out var TPtr, out TSize);
            V = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Vector3>(VPtr, VSize, Allocator.Temp);
            N = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Vector3>(NPtr, VSize, Allocator.Temp);
            F = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<int>(FPtr, 3 * FSize, Allocator.Temp);
            T = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<int>(TPtr, 4 * TSize, Allocator.Temp);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref V, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref N, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref F, AtomicSafetyHandle.Create());
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref T, AtomicSafetyHandle.Create());
#endif
            mesh.SetVertexBufferParams(VSize, BackEnd.VertexBufferLayout);
            mesh.SetIndexBufferParams(3 * FSize, IndexFormat.UInt32);

            mesh.SetVertices(V);
            mesh.SetIndices(F, MeshTopology.Triangles, 0);
            mesh.SetNormals(N);

            mesh.MarkDynamic(); 
            mesh.MarkModified();
            mesh.RecalculateBounds();
            //
            //physicMesh.tets = new int[TSize * 4];
            //physicMesh.tets = T.ToArray();

            var tetrahedronMesh = ScriptableObject.CreateInstance<TetrahedronMesh>();
            tetrahedronMesh.vertices = new Vector3 [VSize];
            tetrahedronMesh.vertices = V.ToArray();
            tetrahedronMesh.faces = new int[FSize * 3];
            tetrahedronMesh.faces = F.ToArray();
            tetrahedronMesh.tets = new int[TSize * 4];
            tetrahedronMesh.tets = T.ToArray();

            ctx.AddObjectToAsset("script object", tetrahedronMesh);


            V.Dispose();
            N.Dispose();
            F.Dispose();
            T.Dispose();
        }
        #endregion
    }

}
