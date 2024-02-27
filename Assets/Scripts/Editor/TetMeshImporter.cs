using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor.AssetImporters;
using UnityEngine;
using UnityEngine.Rendering;


[ScriptedImporter(1, "tetmesh")]
public class TetMeshImporter : ScriptedImporter
{
    public override void OnImportAsset(AssetImportContext ctx)
    {
        #region Create Imported GameObject
        var tetrahedronMesh = ScriptableObject.CreateInstance<TetrahedronMesh>();
        ctx.AddObjectToAsset("main", tetrahedronMesh);
        ctx.SetMainObject(tetrahedronMesh);

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
            
            tetrahedronMesh.vertices = new Vector3[VSize];
            tetrahedronMesh.vertices = V.ToArray();
            tetrahedronMesh.faces = new int[FSize * 3];
            tetrahedronMesh.faces = F.ToArray();
            tetrahedronMesh.tets = new int[TSize * 4];
            tetrahedronMesh.tets = T.ToArray();

            V.Dispose();
            N.Dispose();
            F.Dispose();
            T.Dispose();
        }
        #endregion
    }

}
