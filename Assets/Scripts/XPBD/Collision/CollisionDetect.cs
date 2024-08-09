using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using UnityEngine.Rendering;

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
    public class CollisionDetect : IDisposable
    {
        private CollisionCamera cam1;
        private CollisionCamera cam2;

        private const string tangentVector = "_TangentVector";
        private const string bitangentVector = "_BitangentVector";

        private List<CollisionConstraint> collisions;
        public List<CollisionConstraint> Collisions => collisions;

        // Sub-texture and framebuffer size (only one side since both are square)
        private readonly int subtextureSize = 16;
        private readonly int rendertextureSize;
        private readonly int subTextureRowSize;

        private const REAL nominal_friction = 0.1f;
        private const REAL kA = 1f;
        private const REAL kB = 1f;

        int kernel;
        private ComputeShader computeShader;
        private ComputeBuffer computeBuffer;
        private Vector4 [] FrictionCoefCache;
        AsyncGPUReadbackRequest req;
        bool coefSet;

        //Timers
        Timer renderTimer = new();
        Timer computeCoefTimer2 = new();
        Timer readbackTimer = new();

        public CollisionDetect()
        {
            GameObject[] gameObjects = GameObject.FindGameObjectsWithTag("CollisionCam");
            if (gameObjects.Length != 2)
                Debug.LogWarning("Collision camera should only be two!!! Current: " + gameObjects.Length);
            cam1 = gameObjects[0].GetComponent<CollisionCamera>();
            cam2 = gameObjects[1].GetComponent<CollisionCamera>();

            collisions = new List<CollisionConstraint>();

            // Get framebuffer size and subtexture size
            rendertextureSize = cam1.renderTexture.width;
            subTextureRowSize = rendertextureSize / subtextureSize;
            FrictionCoefCache = new Vector4[subTextureRowSize * subTextureRowSize];
            System.Array.Fill(FrictionCoefCache, Vector4.zero);

            computeShader = Resources.Load<ComputeShader>("FrictionCoef");
            computeBuffer = new ComputeBuffer(FrictionCoefCache.Length, sizeof(float) * 4);

            kernel = computeShader.FindKernel("CSMain");

            computeShader.SetTexture(kernel, "image", cam1.renderTexture);
            computeShader.SetBuffer(kernel, "Result", computeBuffer);
            computeShader.SetVector("Dispatch", new Vector4(subTextureRowSize, subTextureRowSize, 1));
        }
        public void Dispose()
        {
            computeBuffer.Release();
        }

        public void CollectCollision(List<Body> bodies, List<Primitive> primitives, REAL dt)
        {
            collisions.Clear();
            if (bodies == null || primitives == null)
                return;

            if (bodies.Count == 0 || primitives.Count == 0)
                return;

            foreach(var primitive in primitives)
                primitive.UpdateCollisionMaterial();

            foreach(var body in bodies)
            {
                if (!body.EnableContact)
                    continue;

                // Clear body collisions
                body.ClearCollision();

                // Ignore rigid bodies
                if (body.bodyType == Body.BodyType.Rigid)
                    continue;


                foreach(var primitive in primitives)
                {
                    if (body.bodyType == Body.BodyType.Soft)
                        SoftBodyCollision((SoftBody)body, primitive, dt);
                }
            }

            if(Simulation.get.UseTextureFriction && collisions.Count > 0)
            {
                //computeCoefTimer.Tic();
                //CalculateTextureFriction(cam1.renderTexture);
                //computeCoefTimer.Toc();

                renderTimer.Tic();
                RenderContactPatch();
                renderTimer.Toc();

                computeCoefTimer2.Tic();
                CalculateTextureFrictionGPU(cam1.renderTexture);
                computeCoefTimer2.Toc();

            }
            
            if(collisions.Count > 0 && Simulation.get.collisionVerbose)
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
            }

            coefSet = true;
        }

        private void SoftBodyCollision(SoftBody soft, Primitive primitive, REAL dt)
        {
            Geometry geometry = primitive.Geometry;

            for (int i = 0; i < soft.VerticesNum; ++i)
            {
                // Ignore grabbed particle
                if (i == soft.grabId)
                    continue;

                // Predicted position of particle
                REAL3 predPos = soft.Pos[i] + soft.Vel[i] * dt;
                predPos = soft.Pos[i];
                if (geometry.IsInside(predPos))
                {
                    MyCollision newCollision = new(soft, primitive, i);
                    newCollision.q = geometry.ClosestSurfacePoint(predPos, out REAL3 normal);
                    newCollision.N = normal;

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

                    // Render contact patch
                    if (Simulation.get.UseTextureFriction)
                    {
                        
                        // Set collision camera position
                        //cam1.transform.position = soft.Pos[i] - 0.02f * N;
                        //cam2.transform.position = newCollision.q + 0.02f * N;

                        //cam1.transform.LookAt(soft.Pos[i], up);
                        //cam2.transform.LookAt(newCollision.q, up);

                        //Shader.SetGlobalREAL("_Depth", 0.25f);
                        //Shader.SetGlobalVector(tangentVector, tangent);
                        //Shader.SetGlobalVector(bitangentVector, bitangent);

                        //// Set collision camera view rect
                        //int subtextureNum1 = collisions.Count * 2;
                        //int subtextureNum2 = subtextureNum1 + 1;

                        //// Set cam1 viewport
                        //int row1 = subtextureNum1 / subTextureRowSize;
                        //int col1 = subtextureNum1 % subTextureRowSize;
                        //int x1 = subtextureSize * col1;
                        //int y1 = subtextureSize * row1;
                        //cam1.SetViewPort(x1, y1, subtextureSize, subtextureSize);

                        //// Set cam2 viewport
                        //int row2 = subtextureNum2 / subTextureRowSize;
                        //int col2 = subtextureNum2 % subTextureRowSize;
                        //int x2 = subtextureSize * col2;
                        //int y2 = subtextureSize * row2;
                        //cam2.SetViewPort(x2, y2, subtextureSize, subtextureSize);

                        //renderTimer.Resume();

                        //cam1.RenderToTexture();
                        //cam2.RenderToTexture();

                        //renderTimer.Pause();
                    }

                    collisions.Add(newCollision);
                    soft.collisions.Add(newCollision);
                    primitive.collisions.Add(newCollision);
                }
            }

        }
        private void RenderContactPatch()
        {
            cam1.renderTexture.Release();
            for (int i = 0; i < collisions.Count; i++)
            {
                MyCollision collision = (MyCollision)collisions[i];

                //Set collision camera position
                cam1.transform.position = (float3)(collision.body.Pos[collision.index] - 0.02f * collision.N);
                cam2.transform.position = (float3)(collision.q + 0.02f * collision.N);

                // Set camera axis
                Vector3 up = (float3)collision.T;
                Vector3 tangent = (float3)collision.T;
                Vector3 bitangent = (float3)collision.B;

                cam1.transform.LookAt((float3)collision.body.Pos[collision.index], up);
                cam2.transform.LookAt((float3)collision.q, up);

                // Set collision camera view rect
                int subtextureNum1 = i * 2;
                int subtextureNum2 = subtextureNum1 + 1;

                float WH = 1f / subTextureRowSize;
                // Set cam1 viewport
                int row1 = subtextureNum1 / subTextureRowSize;
                int col1 = subtextureNum1 % subTextureRowSize;
                int x1 = subtextureSize * col1;
                int y1 = subtextureSize * row1;
                cam1.SetViewPort((float)col1 / subTextureRowSize, (float)row1 / subTextureRowSize, WH, WH);

                // Set cam2 viewport
                int row2 = subtextureNum2 / subTextureRowSize;
                int col2 = subtextureNum2 % subTextureRowSize;
                int x2 = subtextureSize * col2;
                int y2 = subtextureSize * row2;
                cam2.SetViewPort((float)col2 / subTextureRowSize, (float)row2 / subTextureRowSize, WH, WH);

                /*Shader.SetGlobalFloat("_Depth", 0.25f);
                Shader.SetGlobalVector(tangentVector, tangent);
                Shader.SetGlobalVector(bitangentVector, bitangent);

                Util.SetLayerRecursive(collision.body.gameObject, Layer1);
                Util.SetLayerRecursive(collision.primitive.gameObject, Layer2);

                cam1.RenderToTexture();
                cam2.RenderToTexture();

                Util.SetLayerRecursive(collision.body.gameObject, originLayer);
                Util.SetLayerRecursive(collision.primitive.gameObject, originLayer);*/

                Material mat1 = collision.body.collisionMaterial;
                Material mat2 = collision.primitive.collisionMaterial;

                mat1.SetVector(tangentVector, tangent);
                mat1.SetVector(bitangentVector, bitangent);
                mat2.SetVector(tangentVector, tangent);
                mat2.SetVector(bitangentVector, bitangent);

                cam1.DrawToTexture(collision.body.visualMesh, collision.body.collisionMaterial, collision.body.transform.localToWorldMatrix);
                cam2.DrawToTexture(collision.primitive.mesh, collision.primitive.collisionMaterial, collision.primitive.transform.localToWorldMatrix);
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
            computeShader.Dispatch(kernel, subTextureRowSize, subTextureRowSize, 1);

            req = AsyncGPUReadback.Request(computeBuffer, computeBuffer.stride * collisions.Count * 2, 0);

            coefSet = false;
        }
    }
}

