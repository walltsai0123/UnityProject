using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Assertions;

public class VisMeshData : IDisposable
{
    public NativeArray<Vector3> V;
    public NativeArray<Vector3> N;
    public NativeArray<int> F;

    public readonly int VSize;
    public readonly int FSize;

    private MeshDataNative _native;
    public VisMeshData(TetMesh tetMesh)
    {
        var mesh = tetMesh.mesh;
        VSize = mesh.vertexCount;
        FSize = mesh.triangles.Length / 3;

        Allocate(mesh);
        CopyFrom(mesh);
    }
    public VisMeshData(VisMesh visMesh)
    {
        var mesh = visMesh.mesh;
        VSize = mesh.vertexCount;
        FSize = mesh.triangles.Length / 3;

        Allocate(mesh);
        CopyFrom(mesh);
    }

    private void Allocate(Mesh mesh)
    {
        Assert.IsTrue(VSize > 0 && FSize > 0);
        Assert.IsTrue(!V.IsCreated);

        //var mesh = tetMesh.mesh;

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
                VSize, FSize);
        }
        
    }
    private void CopyFrom(Mesh mesh)
    {
        Assert.IsTrue(V.IsCreated);

        // var mesh = tetMesh.mesh;

        V.CopyFrom(mesh.vertices);
        N.CopyFrom(mesh.normals);
        F.CopyFrom(mesh.triangles);
    }
    public unsafe void ApplyDirty(MeshState* state)
    {
        Assert.IsTrue(VSize == state->VSize && FSize == state->FSize);

        BackEnd.ApplyDirtyVis(state, _native);
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
