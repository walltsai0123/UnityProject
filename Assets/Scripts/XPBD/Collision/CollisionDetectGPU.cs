using System;
using UnityEngine;
using UnityEngine.Assertions;
using Unity.Mathematics;
using System.Linq;
using UnityEngine.Rendering;
using UnityEditor.Build;



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

namespace XPBD.GPU
{
    public class CollisionDetectGPU : MonoBehaviour
    {
        SoftBodySystemGPU softBodySystem;
        [SerializeField] float nominal_friction = 0.1f;
        [SerializeField] float kA = 1f, kB = 1f;
        [SerializeField] ComputeShader computeShader;
        [SerializeField] ComputeShader frictionCoefCS;

        ComputeBuffer collisionBuffer = null;
        ComputeBuffer collisionAppendBuffer = null;
        ComputeBuffer frictionCoefBuffer = null;
        ComputeBuffer counterBuffer = null;
        [SerializeField] CollisionCamera collisionCamera_prefab = null;
        CollisionCamera collisionCamera = null;
        Collision[] collisions = null;
        int numCollisions;

        // Sub-texture and framebuffer size (only one side since both are square)
        [SerializeField] private int subtextureSize = 16;
        private int rendertextureSize;
        private int subTextureRowSize;

        [SerializeField] Material terrainMat;
        [SerializeField] Material arrowMaterial;

        private const string tangentVector = "_TangentVector";
        private const string bitangentVector = "_BitangentVector";

        Timer globalTimer = new();
        // gpu kernels
        struct CollisionKernel
        {
            public const int Detection = 0;
            public const int Copy = 1;
            public const int Solve = 2;
            public const int VelocitySolve = 3;
        };
        struct Collision
        {
            public uint valid;
            public REAL3 q;
            public REAL3 pos;
            public REAL3 N;
            public REAL3 T;
            public REAL3 B;
            REAL3 fn;
            REAL vn_;
        };

        private void OnEnable()
        {
            if(collisionCamera == null)
            {
                collisionCamera = Instantiate<CollisionCamera>(collisionCamera_prefab);
                collisionCamera.transform.SetParent(transform);
            }

            rendertextureSize = collisionCamera.renderTexture.width;
            subTextureRowSize = rendertextureSize / subtextureSize;
        }
        private void OnDisable() {
            ComputeHelper.Release(collisionBuffer, frictionCoefBuffer, collisionAppendBuffer, counterBuffer);
        }
        private void LateUpdate()
        {
            //Draw arrow
            if(collisionAppendBuffer != null)
            {
                arrowMaterial.SetBuffer("collisions", collisionAppendBuffer);
                Graphics.DrawProcedural(arrowMaterial, new Bounds(Vector3.zero, Vector3.one * 1000), MeshTopology.Lines, numCollisions * 2);
            }
        }
        public void CollectCollisions(Terrain terrain, REAL dt)
        {
            int VerticesNum = softBodySystem.VerticesNum;
            TerrainData terrainData = terrain.terrainData;

            if (VerticesNum <= 0)
                return;

            collisionAppendBuffer ??= ComputeHelper.CreateAppendBuffer<Collision>(VerticesNum);
            //collisionBuffer ??= ComputeHelper.CreateStructuredBuffer<Collision>(VerticesNum);
            frictionCoefBuffer ??= ComputeHelper.CreateStructuredBuffer<float4>(VerticesNum);
            counterBuffer ??= new ComputeBuffer(1, sizeof(int), ComputeBufferType.Raw);

            
            ComputeHelper.SetBuffer(computeShader, collisionAppendBuffer, "appendCollisions", CollisionKernel.Detection);
            ComputeHelper.SetBuffer(computeShader, collisionAppendBuffer, "consumeCollisions", CollisionKernel.Copy);
            ComputeHelper.SetBuffer(computeShader, collisionAppendBuffer, "collisions"
                , CollisionKernel.Detection, CollisionKernel.Solve, CollisionKernel.VelocitySolve);
            ComputeHelper.SetBuffer(computeShader, frictionCoefBuffer, "frictionCoef", CollisionKernel.Solve, CollisionKernel.VelocitySolve);
            //ComputeHelper.SetBuffer(computeShader, softBodySystem.PositionBuffer, "pos", CollisionKernel.Detection, CollisionKernel.Solve);
            //ComputeHelper.SetBuffer(computeShader, softBodySystem.PrevPositionBuffer, "prevPos", CollisionKernel.Solve);
            //ComputeHelper.SetBuffer(computeShader, softBodySystem.VelocityBuffer, "vel"
            //    , CollisionKernel.Detection, CollisionKernel.Solve, CollisionKernel.VelocitySolve);
            ComputeHelper.SetBuffer(computeShader, softBodySystem.ParticleBuffer, "particles"
                , CollisionKernel.Detection, CollisionKernel.Solve, CollisionKernel.VelocitySolve);
            //ComputeHelper.SetBuffer(computeShader, softBodySystem.InverseMassBuffer, "invMass", CollisionKernel.Solve);
            ComputeHelper.AssignTexture(computeShader, terrainData.heightmapTexture, "_TerrainHeightMap", CollisionKernel.Detection);
            ComputeHelper.AssignTexture(computeShader, terrain.normalmapTexture, "_TerrainNormalMap", CollisionKernel.Detection);
            
            computeShader.SetFloat("dt", (float)dt);
            computeShader.SetInt("verticesNum", softBodySystem.VerticesNum);
            computeShader.SetVector("terrainOrigin", terrain.GetPosition());
            computeShader.SetVector("terrainSize", terrainData.size);

            globalTimer.Tic();
            collisionAppendBuffer.SetCounterValue(0);
            ComputeHelper.Dispatch(computeShader, softBodySystem.VerticesNum, kernelIndex: CollisionKernel.Detection);

            RenderContactPatches();
        }

        public void CollisionSolve(REAL dt)
        {
            if(numCollisions == 0) return;
            computeShader.SetFloat("dt", (float)dt);
            ComputeHelper.Dispatch(computeShader, numCollisions, kernelIndex: CollisionKernel.Solve);
        }

        public void CollisionVelocitySolve(REAL dt)
        {
            if (numCollisions == 0) return;
            computeShader.SetFloat("dt", (float)dt);
            ComputeHelper.Dispatch(computeShader, numCollisions, kernelIndex: CollisionKernel.VelocitySolve);
        }

        public void SetSoftBodySystem(SoftBodySystemGPU sbs)
        {
            softBodySystem = sbs;
        }
        private void RenderContactPatches()
        {
            collisionCamera.renderTexture.Release();
            Timer timer = new Timer();

            //timer.Tic();
            //numCollisions = 1;
            numCollisions = ComputeHelper.ReadAppendBufferLength(collisionAppendBuffer);
            computeShader.SetInt("numCollisions", numCollisions);

            globalTimer.Toc();
            globalTimer.Report("ReadAppendBufferLength", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            //No collisions
            if (numCollisions == 0)
                return;
            Debug.Log("numCollisions " + numCollisions);
            collisionBuffer?.Release();
            collisionBuffer = ComputeHelper.CreateStructuredBuffer<Collision>(numCollisions);
            ComputeHelper.SetBuffer(computeShader, collisionAppendBuffer, "collisions"
                , CollisionKernel.Copy, CollisionKernel.Solve, CollisionKernel.VelocitySolve);
            
            //ComputeHelper.Dispatch(computeShader, numCollisions, kernelIndex: CollisionKernel.Copy);

            timer.Tic();

            collisions = new Collision[numCollisions];
            //collisionBuffer.GetData(collisions);
            collisionAppendBuffer.GetData(collisions);

            timer.Toc();
            timer.Report("GetData", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);

            timer.Tic();timer.Pause();
            for (int i = 0; i < collisions.Length; i++)
            {
                Collision c = collisions[i];
                //if(c.valid == 0)
                //    continue;

                // Set camera axis
                Vector3 up = (float3)c.T;
                Vector3 tangent = (float3)c.T;
                Vector3 bitangent = (float3)c.B;

                Shader.SetGlobalVector(tangentVector, tangent);
                Shader.SetGlobalVector(bitangentVector, bitangent);

                // Set collision camera view rect
                float WH = 1f / subTextureRowSize;
                int subtextureNum1 = i * 2;
                int subtextureNum2 = subtextureNum1 + 1;

                collisionCamera.transform.position = (float3)(c.q - 0.02f * c.N);
                collisionCamera.transform.LookAt((float3)c.q, up);
                int row1 = subtextureNum1 / subTextureRowSize;
                int col1 = subtextureNum1 % subTextureRowSize;
                collisionCamera.SetViewPort((float)col1 / subTextureRowSize, (float)row1 / subTextureRowSize, WH, WH);
                //collisionCamera.DrawToTexture(softBodySystem.me)

                collisionCamera.transform.position = (float3)(c.q + 0.02f * c.N);
                collisionCamera.transform.LookAt((float3)c.q, up);
                // Set cam2 viewport
                int row2 = subtextureNum2 / subTextureRowSize;
                int col2 = subtextureNum2 % subTextureRowSize;
                collisionCamera.SetViewPort((float)col2 / subTextureRowSize, (float)row2 / subTextureRowSize, WH, WH);
                //collisionCamera.SetViewPort(0,0,1,1);
                collisionCamera.shader = terrainMat.shader;

                

                timer.Resume();
                collisionCamera.RenderToTexture();
                //collisionCamera.DrawToTexture();
                timer.Pause();
                //Simulation.get.pause = true;

                collisionCamera.transform.position = (float3)(c.q - 0.02f * c.N);
                collisionCamera.transform.LookAt((float3)c.q, up);
            }
            timer.Toc();
            timer.Report("Render Contact Patch", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);

            CalculateTextureFrictionGPU(collisionCamera.renderTexture);
        }
        private void CalculateTextureFrictionGPU(RenderTexture renderTexture)
        {
            ComputeBuffer buffer = ComputeHelper.CreateStructuredBuffer<REAL4>(subTextureRowSize * subTextureRowSize);

            ComputeHelper.SetBuffer(frictionCoefCS, buffer, "Result", 0, 1);
            ComputeHelper.AssignTexture(frictionCoefCS, renderTexture, "image", 0);
            frictionCoefCS.SetVector("Dispatch", new Vector4(subTextureRowSize, subTextureRowSize, 1));
            frictionCoefCS.Dispatch(0, subTextureRowSize, subTextureRowSize, 1);

            ComputeHelper.SetBuffer(frictionCoefCS, frictionCoefBuffer, "frictionCoef", 1);
            frictionCoefCS.SetFloat("collisions", frictionCoefBuffer.count);
            frictionCoefCS.SetInt("subtextureSize", subtextureSize);
            frictionCoefCS.SetFloat("nominal_friction", nominal_friction);
            frictionCoefCS.SetFloat("kA", kA);
            frictionCoefCS.SetFloat("kB", kB);
            ComputeHelper.Dispatch(frictionCoefCS, numCollisions, kernelIndex: 1);
            
            buffer.Release();
        }
    }
}