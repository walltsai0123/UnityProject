using System;
using System.Collections.Generic;
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
            tetrahedronMesh.edges = CalculateMeshEdges(tetrahedronMesh.tets).ToArray();

            V.Dispose();
            N.Dispose();
            F.Dispose();
            T.Dispose();
        }
        #endregion
    }

    private List<int> CalculateMeshEdges(int [] tets)
    {
        HashSet<Vector2Int> Set = new HashSet<Vector2Int>();

        for(int i = 0; i < tets.Length; i += 4)
        {
            int [] ids = new int[4];
            ids[0] = tets[i + 0];
            ids[1] = tets[i + 1];
            ids[2] = tets[i + 2];
            ids[3] = tets[i + 3];

            Array.Sort(ids);

            Vector2Int e0 = new(ids[0], ids[1]);
            Vector2Int e1 = new(ids[0], ids[2]);
            Vector2Int e2 = new(ids[0], ids[3]);
            Vector2Int e3 = new(ids[1], ids[2]);
            Vector2Int e4 = new(ids[1], ids[3]);
            Vector2Int e5 = new(ids[2], ids[3]);

            Set.Add(e0);
            Set.Add(e1);
            Set.Add(e2);
            Set.Add(e3);
            Set.Add(e4);
            Set.Add(e5);
        }

        List<int> result = new();

        foreach(var edge in Set)
        {
            result.Add(edge[0]);
            result.Add(edge[1]);
        };

        return result;
    }

}
