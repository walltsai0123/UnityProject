using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Assertions;
using XPBD;

public class TetMeshData : IDisposable
{
    public NativeArray<Vector3> V;
    public NativeArray<int> T;

    public readonly int VSize;
    public readonly int TSize;

    private TetMeshDataNative _native;

    public TetMeshData(PhysicMesh physicMesh)
    {
        var mesh = physicMesh.mesh;
        VSize = mesh.vertexCount;
        TSize = physicMesh.tets.Length / 4;

        Allocate();
        CopyFrom(physicMesh);
    }

    private void Allocate()
    {
        Assert.IsTrue(VSize > 0 && TSize > 0);
        Assert.IsTrue(!V.IsCreated);

        //var mesh = tetMesh.mesh;

        V = new NativeArray<Vector3>(VSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        T = new NativeArray<int>(4 * TSize, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        // Before we can use this we need to add a safety handle (only in the editor)
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref V, AtomicSafetyHandle.Create());
        NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref T, AtomicSafetyHandle.Create());
#endif

        // Store the native pointers int _native
        unsafe
        {
            // NativeArrays will be fixed by default so we can get these pointers only once, not every time we use them
            _native = new TetMeshDataNative(
                (float*)V.GetUnsafePtr(), (int*)T.GetUnsafePtr(), VSize, TSize);
        }
    }

    private void CopyFrom(PhysicMesh physicMesh)
    {
        Assert.IsTrue(V.IsCreated);

        V.CopyFrom(physicMesh.mesh.vertices);
        T.CopyFrom(physicMesh.tets);
    }
    public unsafe void ApplyDirty(TetMeshState* state)
    {
        Assert.IsTrue(VSize == state->VSize && TSize == state->TSize);

        BackEnd.ApplyDirtyTet(state, _native);
    }
    public void ApplyDirtyToMesh(Mesh mesh)
    {
        mesh.SetVertices(V);
        mesh.RecalculateBounds();
    }
    public TetMeshDataNative GetNative()
    {
        return _native;
    }

    public void Dispose()
    {
        if (V.IsCreated) V.Dispose();
        if (T.IsCreated) T.Dispose();
    }
}
