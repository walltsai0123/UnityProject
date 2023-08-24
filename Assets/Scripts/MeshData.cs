using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Assertions;

public class MeshData : IDisposable
{
    public NativeArray<Vector3> V;
    public NativeArray<Vector3> N;
    public NativeArray<Color> C;
    public NativeArray<Vector2> UV;
    public NativeArray<int> F;
    public NativeArray<int> T;

    public readonly int VSize;
    public readonly int FSize;
    public readonly int TSize;

    private MeshDataNative _native;
    public MeshData(TetMesh tetMesh)
    {
        var mesh = tetMesh.mesh;
        VSize = mesh.vertexCount;
        FSize = mesh.triangles.Length / 3;
        TSize = tetMesh.tets.Length / 4;

        Allocate(tetMesh);
        CopyFrom(tetMesh);
    }

    private void Allocate(TetMesh tetMesh)
    {
        Assert.IsTrue(VSize > 0 && FSize > 0 && TSize > 0);
        Assert.IsTrue(!V.IsCreated);

        var mesh = tetMesh.mesh;

        V = new NativeArray<Vector3>(VSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        N = new NativeArray<Vector3>(VSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        C = new NativeArray<Color>(VSize, Allocator.Persistent,
            mesh.colors.Length == 0 ? NativeArrayOptions.UninitializedMemory : NativeArrayOptions.ClearMemory);
        UV = new NativeArray<Vector2>(VSize, Allocator.Persistent,
            mesh.uv.Length == 0 ? NativeArrayOptions.UninitializedMemory : NativeArrayOptions.ClearMemory);
        F = new NativeArray<int>(3 * FSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        T = new NativeArray<int>(4 * TSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        // Before we can use this we need to add a safety handle (only in the editor)
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref V, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref N, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref C, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref UV, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref F, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref T, AtomicSafetyHandle.Create());
#endif

        // Store the native pointers int _native
        unsafe
        {
            // NativeArrays will be fixed by default so we can get these pointers only once, not every time we use them
            _native = new MeshDataNative(
                (float*)V.GetUnsafePtr(), (float*)N.GetUnsafePtr(),
                (float*)C.GetUnsafePtr(), (float*)UV.GetUnsafePtr(),
                (int*)F.GetUnsafePtr(), (int*)T.GetUnsafePtr(),
                VSize, FSize, TSize
                );
        }
    }
    private void CopyFrom(TetMesh tetMesh)
    {
        Assert.IsTrue(V.IsCreated);

        var mesh = tetMesh.mesh;

        V.CopyFrom(mesh.vertices);
        N.CopyFrom(mesh.normals);
        if (mesh.colors.Length > 0)
            C.CopyFrom(mesh.colors);
        if (mesh.uv.Length > 0)
            UV.CopyFrom(mesh.uv);
        F.CopyFrom(mesh.triangles);
        T.CopyFrom(tetMesh.tets);
    }
    public MeshDataNative GetNative()
    {
        return _native;
    }
    public void Dispose()
    {
        if (V.IsCreated) V.Dispose();
        if (N.IsCreated) N.Dispose();
        if (C.IsCreated) C.Dispose();
        if (UV.IsCreated) UV.Dispose();
        if (F.IsCreated) F.Dispose();
        if (T.IsCreated) T.Dispose();
    }
}
