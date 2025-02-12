using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using UnityEngine.Rendering;
using UnityEngine.Assertions;

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
    public class CollisionDetect : MonoBehaviour
    {
        //private CollisionCamera cam1;
        //private CollisionCamera cam2;
        SoftBodySystem softBodySystem;

        [SerializeField] RenderTexture renderTexture;
        [SerializeField] CollisionCamera collisionCamera_prefab = null;
        CollisionCamera collisionCamera = null;

        private const string tangentVector = "_TangentVector";
        private const string bitangentVector = "_BitangentVector";

        private List<MyCollision> collisions;
        public List<MyCollision> Collisions => collisions;

        // Sub-texture and framebuffer size (only one side since both are square)
        private readonly int subtextureSize = 16;
        private int rendertextureSize;
        private int subTextureRowSize;

        [SerializeField] private float nominal_friction = 0.1f;
        [SerializeField] private float kA = 1f;
        [SerializeField] private float kB = 1f;

        [SerializeField] Material terrainMat;

        int kernel;
        private ComputeShader computeShader;
        private ComputeBuffer computeBuffer;
        private Vector4[] FrictionCoefCache;
        AsyncGPUReadbackRequest req;
        bool coefSet;

        //Timers
        readonly Timer renderTimer = new();
        readonly Timer computeCoefTimer2 = new();
        readonly Timer readbackTimer = new();

        private void Awake()
        {
            if (collisionCamera == null)
            {
                collisionCamera = Instantiate(collisionCamera_prefab);
                collisionCamera.transform.SetParent(transform);
                collisionCamera.renderTexture = renderTexture;
            }

            collisions = new List<MyCollision>();

            // Get framebuffer size and subtexture size
            rendertextureSize = renderTexture.width;
            subTextureRowSize = rendertextureSize / subtextureSize;
            FrictionCoefCache = new Vector4[subTextureRowSize * subTextureRowSize];
            Array.Fill(FrictionCoefCache, Vector4.zero);

            computeShader = Resources.Load<ComputeShader>("FrictionCoef");
            computeBuffer = new ComputeBuffer(FrictionCoefCache.Length, sizeof(float) * 4);

            kernel = computeShader.FindKernel("CSMain");

            Assert.IsNotNull(renderTexture);
            computeShader.SetTexture(kernel, "image", renderTexture);
            computeShader.SetBuffer(kernel, "Result", computeBuffer);
            computeShader.SetVector("Dispatch", new Vector4(subTextureRowSize, subTextureRowSize, 1));
        }
        private void OnDestroy()
        {
            Dispose();
        }

        public int collisionindex = 0;
        private void OnDrawGizmos()
        {
            if (collisions != null)
            {
                if (collisions.Count > 0)
                {
                    collisionindex = math.clamp(collisionindex, 0, collisions.Count - 1);

                    MyCollision C = collisions[collisionindex];
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere((float3)C.q, 0.1f);
                    Gizmos.DrawLine((float3)C.q, (float3)(C.q + C.N));
                }
            }
        }
        public void Dispose()
        {
            computeBuffer.Release();
        }

        public void CollectCollision(SoftBodySystem sBS, TerrainSystem terrainSystem, REAL dt)
        {
            softBodySystem = sBS;
            collisions.Clear();
            if (softBodySystem.VerticesNum == 0 || terrainSystem == null)
            {
                Debug.LogWarning("SoftBodySystem has no vertex or terrainSysterm is null");
                return;
            }

            for (int i = 0; i < softBodySystem.VerticesNum; ++i)
            {
                SoftBodyParticle particle = softBodySystem.particles[i];
                // Predicted position of particle
                REAL3 predPos = particle.pos + particle.vel * dt;
                //predPos = particle.pos;

                //REAL x = (predPos.x - terrainPosition.x) / terrainSize.x;
                //REAL z = (predPos.z - terrainPosition.z) / terrainSize.z;

                //REAL sampleHeight = terrain.Terrain.SampleHeight((float3)predPos);

                // skip if not inside terrain
                if (!terrainSystem.IsInside(particle.pos) && !terrainSystem.IsInside(predPos)) continue;

                REAL sampleHeight = terrainSystem.SampleTerrainHeight(predPos);
                if (predPos.y <= sampleHeight)
                {
                    MyCollision newCollision = new(i)
                    {
                        q = new REAL3(predPos.x, sampleHeight, predPos.z),
                        //N = (float3)terrainData.GetInterpolatedNormal((float)x, (float)z)
                        N = terrainSystem.SampleTerrainNormal(predPos)
                    };

                    // Calculate the Normal, Tangent, Bitangent vector of contact frame
                    REAL3 N = newCollision.N;
                    REAL3 up = math.cross(newCollision.N, new REAL3(1, 0, 0));
                    if (math.length(up) < Util.EPSILON)
                    {
                        up = -math.cross(newCollision.N, new REAL3(0, 0, -1));
                    }
                    math.normalize(up);

                    REAL3 tangent = up;
                    REAL3 bitangent = math.cross(N, tangent);

                    newCollision.T = up;
                    newCollision.B = bitangent;

                    collisions.Add(newCollision);
                    //soft.collisions.Add(newCollision);

                    //terrain.SetupMaterial(newCollision);
                }
            }


            if (collisions.Count == 0)
                return;

            if (Simulation.get.UseTextureFriction)
            {
                renderTimer.Tic();
                RenderContactPatch();
                renderTimer.Toc();

                computeCoefTimer2.Tic();
                CalculateTextureFrictionGPU(renderTexture);
                computeCoefTimer2.Toc();
            }

            if (Simulation.get.collisionVerbose)
            {
                Debug.Log("Collision count: " + collisions.Count);
                Debug.Log("Average render time: " + renderTimer.Duration() * 0.001f / collisions.Count + "ms");
                renderTimer.Report("Render contact patch", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
                computeCoefTimer2.Report("Compute Coef2", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            }
        }
        public void SetFrictionCoef()
        {
            if (coefSet)
                return;

            readbackTimer.Tic();

            if (!req.done)
            {
                req.WaitForCompletion();
            }

            if (req.hasError)
            {
                computeBuffer.GetData(FrictionCoefCache);
            }
            else
            {
                FrictionCoefCache = req.GetData<Vector4>().ToArray();
            }
            readbackTimer.Toc();
            //readbackTimer.Report("read back: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            for (int i = 0; i < collisions.Count; ++i)
            {
                REAL4 mu1 = (float4)FrictionCoefCache[2 * i];
                REAL4 mu2 = (float4)FrictionCoefCache[2 * i + 1];
                float validPixels = subtextureSize * subtextureSize;

                mu1 /= validPixels;
                mu2 /= validPixels;

                collisions[i].frictionCoef = kA * mu1 + kB * mu2;
                collisions[i].frictionCoef += nominal_friction;

                //Debug.Log(collisions[i].frictionCoef);
            }

            coefSet = true;
        }

        public void SolveCollision(REAL dt)
        {
            SetFrictionCoef();

            foreach (var collision in  collisions)
            {
                collision.SolveCollision(softBodySystem, dt);
            }
        }

        public void VelocitySolve(REAL dt)
        {
            foreach (var collision in collisions)
            {
                collision.VelocitySolve(softBodySystem, dt);
            }
        }

        private void SoftBodyCollision(SoftBody soft, Primitive primitive, REAL dt)
        {
            //Geometry geometry = primitive.Geometry;

            //for (int i = 0; i < soft.VerticesNum; ++i)
            //{
            //    // Ignore grabbed particle
            //    if (i == soft.grabId)
            //        continue;

            //    // Predicted position of particle
            //    REAL3 predPos = soft.Pos[i] + soft.Vel[i] * dt;
            //    predPos = soft.Pos[i];
            //    if (geometry.IsInside(predPos))
            //    {
            //        MyCollision newCollision = new(soft, primitive, i);
            //        newCollision.q = geometry.ClosestSurfacePoint(predPos, out REAL3 normal);
            //        newCollision.N = normal;

            //        // Calculate the Normal, Tangent, Bitangent vector of contact frame
            //        REAL3 N = newCollision.N;
            //        REAL3 up = math.cross(newCollision.N, new REAL3(1, 0, 0));
            //        if (math.length(up) < Util.EPSILON)
            //        {
            //            up = -math.cross(newCollision.N, new REAL3(0, 0, -1));
            //        }
            //        math.normalize(up);

            //        REAL3 tangent = up;
            //        REAL3 bitangent = math.cross(N, tangent);

            //        newCollision.T = up;
            //        newCollision.B = bitangent;

            //        // Render contact patch
            //        if (Simulation.get.UseTextureFriction)
            //        {
                        
            //            // Set collision camera position
            //            //cam1.transform.position = soft.Pos[i] - 0.02f * N;
            //            //cam2.transform.position = newCollision.q + 0.02f * N;

            //            //cam1.transform.LookAt(soft.Pos[i], up);
            //            //cam2.transform.LookAt(newCollision.q, up);

            //            //Shader.SetGlobalREAL("_Depth", 0.25f);
            //            //Shader.SetGlobalVector(tangentVector, tangent);
            //            //Shader.SetGlobalVector(bitangentVector, bitangent);

            //            //// Set collision camera view rect
            //            //int subtextureNum1 = collisions.Count * 2;
            //            //int subtextureNum2 = subtextureNum1 + 1;

            //            //// Set cam1 viewport
            //            //int row1 = subtextureNum1 / subTextureRowSize;
            //            //int col1 = subtextureNum1 % subTextureRowSize;
            //            //int x1 = subtextureSize * col1;
            //            //int y1 = subtextureSize * row1;
            //            //cam1.SetViewPort(x1, y1, subtextureSize, subtextureSize);

            //            //// Set cam2 viewport
            //            //int row2 = subtextureNum2 / subTextureRowSize;
            //            //int col2 = subtextureNum2 % subTextureRowSize;
            //            //int x2 = subtextureSize * col2;
            //            //int y2 = subtextureSize * row2;
            //            //cam2.SetViewPort(x2, y2, subtextureSize, subtextureSize);

            //            //renderTimer.Resume();

            //            //cam1.RenderToTexture();
            //            //cam2.RenderToTexture();

            //            //renderTimer.Pause();
            //        }

            //        collisions.Add(newCollision);
            //        soft.collisions.Add(newCollision);
            //        primitive.collisions.Add(newCollision);
            //    }
            //}

        }

        private void RenderContactPatch()
        {
            renderTexture.Release();
            var terrain = Simulation.get.terrain;
            for (int i = 0; i < collisions.Count; i++)
            {
                MyCollision collision = collisions[i];

                // contact frame axis
                Vector3 up = (float3)collision.T;
                Vector3 tangent = (float3)collision.T;
                Vector3 bitangent = (float3)collision.B;

                // Set material uniform variable
                SoftBody body = softBodySystem.GetParticleBody(collision.index);
                Material mat1 = body.collisionMaterial;
                mat1.SetVector(tangentVector, tangent);
                mat1.SetVector(bitangentVector, bitangent);

                // viewport width/height
                float WH = 1f / subTextureRowSize;

                //Object 1
                //Set collision camera position
                collisionCamera.transform.position = (float3)(collision.q - 0.02f * collision.N);
                // Set camera axis
                collisionCamera.transform.LookAt((float3)collision.q, up);
                // Set collision camera view rect
                int subtextureNum1 = i * 2;
                // Set cam viewport
                int row1 = subtextureNum1 / subTextureRowSize;
                int col1 = subtextureNum1 % subTextureRowSize;
                collisionCamera.SetViewPort((float)col1 / subTextureRowSize, (float)row1 / subTextureRowSize, WH, WH);
                //collisionCamera.DrawToTexture(body.visualMesh, body.collisionMaterial, body.transform.localToWorldMatrix);

                //Object 2
                //Set collision camera position
                collisionCamera.transform.position = (float3)(collision.q + 0.02f * collision.N);
                // Set camera axis
                collisionCamera.transform.LookAt((float3)collision.q, up);
                // Set collision camera view rect
                int subtextureNum2 = subtextureNum1 + 1;
                // Set cam viewport
                int row2 = subtextureNum2 / subTextureRowSize;
                int col2 = subtextureNum2 % subTextureRowSize;
                collisionCamera.SetViewPort((float)col2 / subTextureRowSize, (float)row2 / subTextureRowSize, WH, WH);

                terrain.SetupMaterial(collision);
                //collisionCamera.DrawToTexture(terrain.mesh2, terrain.CollisionMaterial, terrain.transform.localToWorldMatrix, true);
                
            }

        }
        private void CalculateTextureFriction(RenderTexture renderTexture)
        {
            // Copy rendertexture to texture2D
            RenderTexture.active = renderTexture;
            Texture2D image = new(rendertextureSize, rendertextureSize);
            image.ReadPixels(new Rect(0, 0, rendertextureSize, rendertextureSize), 0, 0, false);
            RenderTexture.active = null;

            System.Array.Fill(FrictionCoefCache, Vector4.zero);

            for (int i = 0; i < collisions.Count; ++i)
            {
                // Set start pixel
                int subtextureNum1 = i * 2;
                int subtextureNum2 = subtextureNum1 + 1;
                int row1 = subtextureNum1 / subTextureRowSize;
                int col1 = subtextureNum1 % subTextureRowSize;
                int row2 = subtextureNum2 / subTextureRowSize;
                int col2 = subtextureNum2 % subTextureRowSize;
                int startW1 = subtextureSize * col1;
                int startH1 = subtextureSize * row1;
                int startW2 = subtextureSize * col2;
                int startH2 = subtextureSize * row2;

                // Sub-texture
                REAL4 mu1 = REAL4.zero;
                REAL4 mu2 = REAL4.zero;
                int validPixels = 0;

                for (int w = 0; w < subtextureSize; ++w)
                {
                    for(int h = 0; h < subtextureSize; ++h)
                    {
                        Vector4 color1 = image.GetPixel(w + startW1, h + startH1);
                        Vector4 color2 = image.GetPixel(w + startW2, h + startH2);
                        //if (Vector4.Magnitude(color1) < Util.EPSILON || Vector4.Magnitude(color2) < Util.EPSILON)
                        //    continue;
                        //Debug.Log("color1: " + color1);
                        //Debug.Log("color2: " + color2.ToString("F4"));

                        mu1 += (float4)color1;
                        mu2 += (float4)color2;
                        validPixels++;
                    }
                }

                if (validPixels == 0)
                {
                    collisions[i].frictionCoef = REAL4.zero + nominal_friction;
                    continue;
                }

                FrictionCoefCache[2 * i] = (float4)mu1;
                FrictionCoefCache[2 * i + 1] = (float4)mu2;

                mu1 /= validPixels;
                mu2 /= validPixels;

                collisions[i].frictionCoef = kA * mu1 + kB * mu2;
                collisions[i].frictionCoef += nominal_friction;

                //Debug.Log("mu1: " + mu1);
                //Debug.Log("mu2: " + mu2.ToString("F5"));
                //Debug.Log(mu2.y);
                //Debug.Log(mu2.z);
                //Debug.Log(mu2.w);
            }
            UnityEngine.Object.Destroy(image);
        }

        private void CalculateTextureFrictionGPU(RenderTexture renderTexture)
        {
            computeShader.SetFloat("nominal_friction", nominal_friction);
            computeShader.SetFloat("kA", kA);
            computeShader.SetFloat("kB", kB);
            computeShader.Dispatch(kernel, subTextureRowSize, subTextureRowSize, 1);

            req = AsyncGPUReadback.Request(computeBuffer, computeBuffer.stride * collisions.Count * 2, 0);

            coefSet = false;
        }
    }
}

