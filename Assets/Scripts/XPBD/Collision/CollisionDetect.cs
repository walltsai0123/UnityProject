using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using UnityEditor;

namespace XPBD
{
    public class CollisionDetect
    {
        private CollisionCamera cam1;
        private CollisionCamera cam2;
        private int Layer1;
        private int Layer2;
        private int originLayer;

        private const string tangentVector = "_TangentVector";
        private const string bitangentVector = "_BitangentVector";

        private List<CollisionConstraint> collisions;
        public List<CollisionConstraint> Collisions => collisions;

        // Sub-texture and framebuffer size (only one side since both are square)
        private readonly int subtextureSize = 16;
        private readonly int rendertextureSize;
        private readonly int subTextureRowSize;

        private const float nominal_friction = 0.1f;


        //Timers
        Timer renderTimer;
        Timer computeCoefTimer;

        public CollisionDetect()
        {
            GameObject[] gameObjects = GameObject.FindGameObjectsWithTag("CollisionCam");
            if (gameObjects.Length != 2)
                Debug.LogWarning("Collision camera should only be two!!! Current: " + gameObjects.Length);
            cam1 = gameObjects[0].GetComponent<CollisionCamera>();
            cam2 = gameObjects[1].GetComponent<CollisionCamera>();

            collisions = new List<CollisionConstraint>();

            // Set camera culling mask
            Layer1 = LayerMask.NameToLayer("CollisionObject1");
            Layer2 = LayerMask.NameToLayer("CollisionObject2");
            originLayer = LayerMask.NameToLayer("Default");

            cam1.layer = Layer1;
            cam2.layer = Layer2;

            // Get framebuffer size and subtexture size
            rendertextureSize = cam1.renderTexture.width;
            subTextureRowSize = rendertextureSize / subtextureSize;

            //Timer
            renderTimer = new();
            computeCoefTimer = new();
        }

        public void CollectCollision(List<Body> bodies, List<Primitive> primitives, float dt)
        {
            collisions.Clear();
            if (bodies == null || primitives == null)
                return;

            if (bodies.Count == 0 || primitives.Count == 0)
                return;

            renderTimer.Tic(); renderTimer.Pause();
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

            renderTimer.Toc();

            computeCoefTimer.Tic();
            //CalculateTextureFriction(cam1.renderTexture);
            computeCoefTimer.Toc();

            //renderTimer.Report("Render contact patch", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
            //computeCoefTimer.Report("Compute Coef", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
        }

        private void SoftBodyCollision(SoftBody soft, Primitive primitive, float dt)
        {
            Bounds bounds = primitive.collider.bounds;
            Geometry geometry = primitive.Geometry;

            Util.SetLayerRecursive(soft.gameObject, Layer1);
            Util.SetLayerRecursive(primitive.gameObject, Layer2);

            for (int i = 0; i < soft.VerticesNum; ++i)
            {
                // Ignore grabbed particle
                if (i == soft.grabId)
                    continue;

                // Predicted position of particle
                float3 predPos = soft.Pos[i] + soft.Vel[i] * dt;

                if(geometry.IsInside(predPos))
                //if (Util.IsInsideCollider(primitive.collider, predPos))
                {
                    MyCollision newCollision = new(soft, primitive, i);
                    //if (Util.IsInsideCollider(primitive.collider, soft.Pos[i]))
                    //{
                    //    newCollision.N = primitive.defaultNormal;
                    //    float3 pointOutside = Vector3.Dot(primitive.defaultNormal, primitive.collider.bounds.size) * primitive.defaultNormal;
                    //    pointOutside += soft.Pos[i];
                    //    newCollision.q = primitive.collider.ClosestPoint(pointOutside);
                    //}
                    //else
                    //{
                    //    float3 direction = predPos - soft.Pos[i];
                    //    Ray ray = new Ray(soft.Pos[i], direction);
                    //    RaycastHit hit;
                    //    primitive.collider.Raycast(ray, out hit, math.length(direction));
                    //    newCollision.q = hit.point;
                    //    newCollision.N = hit.normal;
                    //}
                    float3 direction = predPos - soft.Pos[i];
                    Ray ray = new Ray(soft.Pos[i], direction);
                    if(false)
                    //if (geometry.Raycast(ray, out RaycastHit hit, math.length(direction)))
                    {
                        //newCollision.q = hit.point;
                        //newCollision.N = hit.normal;
                    }
                    else
                    {
                        newCollision.q = geometry.ClosestSurfacePoint(predPos, out Vector3 normal);
                        newCollision.N = normal;
                    }
                    
                    // Calculate the Normal, Tangent, Bitangent vector of contact frame
                    float3 N = newCollision.N;
                    float3 up = math.cross(newCollision.N, new float3(1, 0, 0));
                    if (math.length(up) < Util.EPSILON)
                    {
                        up = -math.cross(newCollision.N, new float3(0, 0, -1));
                    }
                    math.normalize(up);

                    Vector3 tangent = up;
                    Vector3 bitangent = Vector3.Cross(N, tangent);

                    newCollision.T = up;
                    newCollision.B = bitangent;

                    
                    // Set collision camera position
                    cam1.transform.position = soft.Pos[i] - 0.02f * N;
                    cam2.transform.position = newCollision.q + 0.02f * N;

                    cam1.transform.LookAt(soft.Pos[i], up);
                    cam2.transform.LookAt(newCollision.q, up);

                    Shader.SetGlobalVector(tangentVector, tangent);
                    Shader.SetGlobalVector(bitangentVector, bitangent);

                    // Set collision camera view rect
                    int subtextureNum1 = collisions.Count * 2;
                    int subtextureNum2 = subtextureNum1 + 1;

                    // Set cam1 viewport
                    int row1 = subtextureNum1 / subTextureRowSize;
                    int col1 = subtextureNum1 % subTextureRowSize;
                    int x1 = subtextureSize * col1;
                    int y1 = subtextureSize * row1;
                    cam1.SetViewPort(x1, y1, subtextureSize, subtextureSize);

                    // Set cam2 viewport
                    int row2 = subtextureNum2 / subTextureRowSize;
                    int col2 = subtextureNum2 % subTextureRowSize;
                    int x2 = subtextureSize * col2;
                    int y2 = subtextureSize * row2;
                    cam2.SetViewPort(x2, y2, subtextureSize, subtextureSize);

                    renderTimer.Resume();

                    cam1.RenderToTexture();
                    cam2.RenderToTexture();

                    renderTimer.Pause();
                    //float4 coefs = CalculateTextureFriction(cam1.renderTexture, cam2.renderTexture);
                    //
                    //newCollision.frictionCoef = coefs + 0.1f;

                    //newCollision.frictionCoef = 0.4f;

                    collisions.Add(newCollision);
                }
            }

            Util.SetLayerRecursive(soft.gameObject, originLayer);
            Util.SetLayerRecursive(primitive.gameObject, originLayer);
        }

        private void CalculateTextureFriction(RenderTexture renderTexture)
        {
            // Copy rendertexture to texture2D
            RenderTexture.active = renderTexture;
            Texture2D image = new(rendertextureSize, rendertextureSize);
            image.ReadPixels(new Rect(0, 0, rendertextureSize, rendertextureSize), 0, 0, false);
            RenderTexture.active = null;

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
                Vector4 mu1 = Vector4.zero;
                Vector4 mu2 = Vector4.zero;
                int validPixels = 0;

                for (int w = 0; w < subtextureSize; ++w)
                {
                    for(int h = 0; h < subtextureSize; ++h)
                    {
                        Vector4 color1 = image.GetPixel(w + startW1, h + startH1);
                        Vector4 color2 = image.GetPixel(w + startW2, h + startH2);
                        //if (Vector4.Magnitude(color1) < Util.EPSILON || Vector4.Magnitude(color2) < Util.EPSILON)
                        //    continue;

                        mu1 += color1;
                        mu2 += color2;
                        validPixels++;
                    }
                }

                if (validPixels == 0)
                {
                    collisions[i].frictionCoef = float4.zero + nominal_friction;
                    continue;
                }

                mu1 /= validPixels;
                mu2 /= validPixels;

                collisions[i].frictionCoef = mu1 + 4 * mu2;
                collisions[i].frictionCoef += nominal_friction;
            }
        }

    }
}

