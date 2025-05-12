using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using System.Drawing;


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

        [SerializeField, Range(0,1)] float defaultFrictionCoefficient;
        [SerializeField] private float nominal_friction = 0.1f;
        [SerializeField] private float kA = 1f;
        [SerializeField] private float kB = 1f;

        [SerializeField] Material terrainMat;

        [Range(0f, 1f)] public float raycastDistance = 0.075f;
        [Range(0f, 1f)] public float offsetRay = 0.04f;

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

        public void Dispose()
        {
            computeBuffer.Release();
        }
        public void CollectCollision(SoftBodySystem sBS, TerrainSystem terrainSystem, REAL dt)
        {
            if(sBS == null)
            {
                Debug.LogWarning("SoftBodySystem is null");
                return;
            }
            if(terrainSystem == null)
            {
                Debug.LogWarning("TerrainSystem is null");
                return;
            }

            softBodySystem = sBS;
            collisions.Clear();
            for (int i = 0; i < softBodySystem.softBodies.Count; i++)
                softBodySystem.softBodies[i].collisions.Clear();
            for (int i = 0; i < terrainSystem.Terrains.Count; i++)
                terrainSystem.Terrains[i].collisions.Clear();

            for (int i = 0; i < softBodySystem.VerticesNum; ++i)
            {
                SoftBodyParticle particle = softBodySystem.particles[i];
                SoftBody softBody = softBodySystem.GetParticleBody(i);
                // Predicted position of particle
                REAL3 predPos = particle.pos + particle.vel * dt;

                MyTerrain myTerrain = terrainSystem.GetTerrain((float3)predPos);
                if (myTerrain == null)
                    continue;
                // Continous handling
                REAL3 dir = math.normalizesafe(particle.vel, new REAL3(0, -1, 0));
                Ray ray = new((float3)(particle.pos - dir * offsetRay), (float3)dir);
                if (myTerrain.RayCast(ray, out RaycastHit hit, (float)math.length(particle.vel * dt) + raycastDistance))
                {
                    AddCollision(i, (float3)hit.point, softBody, myTerrain);
                    continue;
                }

                // Static handling if continous handling didn't detect collision
                REAL surfaceHeight = myTerrain.SampleHeight((float3)predPos);
                if (predPos.y < surfaceHeight)
                {
                    AddCollision(i, new REAL3(predPos.x, surfaceHeight, predPos.z), softBody, myTerrain);
                }
            }
        }
        public void CollectCollision(SoftBodySystem sBS, MyTerrain myTerrain, REAL dt)
        {
            softBodySystem = sBS;
            collisions.Clear();
            for (int i = 0; i < softBodySystem.softBodies.Count; i++)
                softBodySystem.softBodies[i].collisions.Clear();
            for (int i = 0; i < softBodySystem.VerticesNum; ++i)
            {
                SoftBodyParticle particle = softBodySystem.particles[i];
                SoftBody softBody = softBodySystem.GetParticleBody(i);

                // Predicted position of particle
                REAL3 predPos = particle.pos + particle.vel * dt;
                // Continous handling
                REAL3 dir = math.normalizesafe(particle.vel, new REAL3(0, -1, 0));
                Ray ray = new ((float3)(particle.pos - dir * offsetRay), (float3)dir);
                if(myTerrain.RayCast(ray, out RaycastHit hit, (float)math.length(particle.vel * dt) + raycastDistance))
                {
                    AddCollision(i, (float3)hit.point, softBody, myTerrain);
                    continue;
                }

                // Static handling if continous handling didn't detect collision
                REAL surfaceHeight = myTerrain.SampleHeight((float3)predPos);
                if(predPos.y < surfaceHeight)
                {
                    AddCollision(i, new REAL3(predPos.x, surfaceHeight, predPos.z), softBody, myTerrain);
                }
            }
        }
        void AddCollision(int index, REAL3 point, SoftBody softBody, MyTerrain inputTerrain)
        {
            MyCollision newCollision = new(index)
            {
                frictionCoef = defaultFrictionCoefficient,
                q = (float3)point,
                N = inputTerrain.SampleNormal((float3)point),
                //N = new float3(0,1,0),
                softBody = softBody,
                terrain = inputTerrain
            };

            // Calculate the Normal, Tangent, Bitangent vector of contact frame
            REAL3 N = newCollision.N;
            REAL3 tangent = math.normalizesafe(softBody.forwardDir - math.dot(N, softBody.forwardDir) * N, 0);
            if(math.lengthsq(tangent) < Util.EPSILON_SQUARE)
            {
                tangent = math.cross(N, new REAL3(0, 0, 1));
                if (math.lengthsq(tangent) < Util.EPSILON_SQUARE)
                    tangent = math.cross(N, new REAL3(1,0,0));
            }
            tangent = math.normalize(tangent);

            newCollision.T = tangent;
            newCollision.B = math.cross(N, tangent);

            collisions.Add(newCollision);

            SoftBody collisionBody = softBodySystem.GetParticleBody(index);
            collisionBody.collisions.Add(newCollision);
            inputTerrain.collisions.Add(newCollision);
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
        private void RenderContactPatch()
        {
            renderTexture.Release();
            var terrain = Simulation.get.myTerrain;
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

                //terrain.SetupMaterial(collision);
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

