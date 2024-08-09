using System;
using UnityEngine;

#if USE_FLOAT
using REAL = System.Single;
using REAL2 = Unity.Mathematics.float2;
using REAL3 = Unity.Mathematics.float3;
using REAL4 = Unity.Mathematics.float4;
using REAL2x2 = Unity.Mathematics.float2x2;
using REAL3x3 = Unity.Mathematics.float3x3;
using REAL3x4 = Unity.Mathematics.float3x4;
#else
using REAL = System.Double;
using REAL2 = Unity.Mathematics.double2;
using REAL3 = Unity.Mathematics.double3;
using REAL4 = Unity.Mathematics.double4;
using REAL2x2 = Unity.Mathematics.double2x2;
using REAL3x3 = Unity.Mathematics.double3x3;
using REAL3x4 = Unity.Mathematics.double3x4;
#endif

namespace XPBD
{
    public class CollisionDetectGPU : MonoBehaviour
    {
        [SerializeField] ComputeShader computeShader;

        ComputeBuffer collisionBuffer = null;

        struct Collision
        {
            public uint valid;
            public REAL3 q;
            public REAL3 N;
            public REAL3 T;
            public REAL3 B;
            REAL3 fn;
            REAL vn_;
        };

        private void OnEnable()
        {
            
        }
        private void OnDisable() {
            collisionBuffer?.Release(); 
        }
        private void OnDrawGizmos()
        {
            if (collisionBuffer == null)
                return;
        }
        public void CollectCollisions(SoftBodySystem softBodySystem, Terrain terrain, REAL dt)
        {
            int VerticesNum = softBodySystem.VerticesNum;
            TerrainData terrainData = terrain.terrainData;

            collisionBuffer ??= ComputeHelper.CreateStructuredBuffer<Collision>(VerticesNum);

            ComputeHelper.SetBuffer(computeShader, collisionBuffer, "collisions", 0, 1, 2);
            ComputeHelper.SetBuffer(computeShader, softBodySystem.PositionBuffer, "pos", 0, 1);
            ComputeHelper.SetBuffer(computeShader, softBodySystem.PrevPositionBuffer, "prevPos", 1);
            ComputeHelper.SetBuffer(computeShader, softBodySystem.VelocityBuffer, "vel", 0, 1, 2);
            ComputeHelper.SetBuffer(computeShader, softBodySystem.InverseMassBuffer, "invMass", 1);
            ComputeHelper.AssignTexture(computeShader, terrainData.heightmapTexture, "_TerrainHeightMap", 0);

            computeShader.SetFloat("dt", (float)dt);
            computeShader.SetInt("verticesNum", softBodySystem.VerticesNum);
            computeShader.SetVector("terrainOrigin", terrain.GetPosition());
            computeShader.SetVector("terrainSize", terrainData.size);
            
            ComputeHelper.Dispatch(computeShader, softBodySystem.VerticesNum, kernelIndex: 0);

            return;
        }

        public void CollisionSolve(REAL dt)
        {
            computeShader.SetFloat("dt", (float)dt);
            ComputeHelper.Dispatch(computeShader, collisionBuffer.count, kernelIndex: 1);
            return;
        }

        public void CollisionVelocitySolve(REAL dt)
        {
            computeShader.SetFloat("dt", (float)dt);
            ComputeHelper.Dispatch(computeShader, collisionBuffer.count, kernelIndex: 2);
        }
    }
}