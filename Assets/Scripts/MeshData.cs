using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Assertions;

public class MeshData : IDisposable
{
    public NativeArray<Vector3> V;
    public NativeArray<Vector3> N;
    public NativeArray<int> F;

    public Vector3 com;
    public readonly int VSize;
    public readonly int FSize;

    public int materialType;

    private MeshDataNative _native;
    public MeshData(TetMesh tetMesh)
    {
        var mesh = tetMesh.mesh;
        VSize = mesh.vertexCount;
        FSize = mesh.triangles.Length / 3;
        com = tetMesh.transform.position;
        materialType = ((int)tetMesh.materialType);

        Allocate(tetMesh);
        CopyFrom(tetMesh);

        Debug.Log(materialType);
    }

    private void Allocate(TetMesh tetMesh)
    {
        Assert.IsTrue(VSize > 0 && FSize > 0);
        Assert.IsTrue(!V.IsCreated);

        var mesh = tetMesh.mesh;

        V = new NativeArray<Vector3>(VSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        N = new NativeArray<Vector3>(VSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        F = new NativeArray<int>(3 * FSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        // Before we can use this we need to add a safety handle (only in the editor)
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref V, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref N, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref F, AtomicSafetyHandle.Create());
#endif

        // Store the native pointers int _native
        unsafe
        {
            // NativeArrays will be fixed by default so we can get these pointers only once, not every time we use them
            _native = new MeshDataNative(
                (float*)V.GetUnsafePtr(), (float*)N.GetUnsafePtr(), (int*)F.GetUnsafePtr(),
                com, tetMesh.mass, 
                materialType, tetMesh.mu, tetMesh.lambda,
                VSize, FSize
                );
        }
    }
    private void CopyFrom(TetMesh tetMesh)
    {
        Assert.IsTrue(V.IsCreated);

        var mesh = tetMesh.mesh;

        V.CopyFrom(mesh.vertices);
        N.CopyFrom(mesh.normals);
        F.CopyFrom(mesh.triangles);
    }
    public unsafe void ApplyDirty(MeshState* state)
    {
        Assert.IsTrue(VSize == state->VSize && FSize == state->FSize);

        BackEnd.ApplyDirty(state, _native);
        com = state->com;
    }
    public void ApplyDirtyToMesh(Mesh mesh)
    {
        mesh.SetVertices(V);
        mesh.RecalculateBounds();
        
        mesh.SetNormals(N);
    }
    public MeshDataNative GetNative()
    {
        return _native;
    }
    public void Dispose()
    {
        if (V.IsCreated) V.Dispose();
        if (N.IsCreated) N.Dispose();
        if (F.IsCreated) F.Dispose();
    }
}
