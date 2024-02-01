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

        private readonly Vector4 blackColor = Color.black;

        private const string tangentVector = "_TangentVector";
        private const string bitangentVector = "_BitangentVector";

        private List<CollisionConstraint> collisions;
        public List<CollisionConstraint> Collisions => collisions;
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

            Debug.Log(cam1);
            Debug.Log(cam2);
        }

        public void CollectCollision(List<Body> bodies, List<Primitive> primitives, float dt)
        {
            collisions.Clear();
            if (bodies == null || primitives == null)
                return;

            if (bodies.Count == 0 || primitives.Count == 0)
                return;

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
                        collisions.AddRange(SoftBodyCollision((SoftBody)body, primitive, dt));
                }
            }
        }

        private List<CollisionConstraint> SoftBodyCollision(SoftBody soft, Primitive primitive, float dt)
        {
            Bounds bounds = primitive.collider.bounds;
            List<CollisionConstraint> newList = new();

            for (int i = 0; i < soft.VerticesNum; ++i)
            {
                if (i == soft.grabId)
                    continue;
                float3 predPos = soft.Pos[i] + soft.Vel[i] * dt;
                if (Util.IsInsideCollider(primitive.collider, predPos))
                {
                    MyCollision newCollision = new(soft, primitive, i);
                    if (Util.IsInsideCollider(primitive.collider, soft.Pos[i]))
                    {
                        newCollision.N = primitive.defaultNormal;
                        float3 pointOutside = Vector3.Dot(primitive.defaultNormal, primitive.collider.bounds.size) * primitive.defaultNormal;
                        pointOutside += soft.Pos[i];
                        newCollision.q = primitive.collider.ClosestPoint(pointOutside);
                    }
                    else
                    {
                        float3 direction = predPos - soft.Pos[i];
                        Ray ray = new Ray(soft.Pos[i], direction);
                        RaycastHit hit;
                        primitive.collider.Raycast(ray, out hit, math.length(direction));
                        newCollision.q = hit.point;
                        newCollision.N = hit.normal;
                    }
                    

                    float3 N = newCollision.N;
                    float3 up = math.cross(newCollision.N, new float3(1, 0, 0));
                    if (math.length(up) < Util.EPSILON)
                    {
                        up = -math.cross(newCollision.N, new float3(0, 0, -1));
                    }
                    math.normalize(up);

                    cam1.transform.position = soft.Pos[i] - 0.02f * N;
                    cam2.transform.position = newCollision.q + 0.02f * N;

                    cam1.transform.LookAt(soft.Pos[i], up);
                    cam2.transform.LookAt(newCollision.q, up);

                    Util.SetLayerRecursive(soft.gameObject, Layer1);
                    Util.SetLayerRecursive(primitive.gameObject, Layer2);

                    Vector3 tangent = up;
                    Vector3 bitangent = Vector3.Cross(N, tangent);

                    newCollision.T = up;
                    newCollision.B = bitangent;

                    //Debug.Log("normal " + N);
                    //Debug.Log("tangent " + tangent);
                    //Debug.Log("bitangent " + bitangent);

                    Shader.SetGlobalVector(tangentVector, tangent);
                    Shader.SetGlobalVector(bitangentVector, bitangent);

                    cam1.RenderToTexture();
                    cam1.Render();
                    cam1.RenderToDisplay();

                    cam2.RenderToTexture();
                    cam2.Render();
                    cam2.RenderToDisplay();


                    float4 coefs = CalculateTextureFriction(cam1.renderTexture, cam2.renderTexture);

                    newCollision.frictionCoef = coefs + 0.1f;

                    newList.Add(newCollision);
                }
            }

            return newList;
        }

        private Vector4 CalculateTextureFriction(RenderTexture textureA, RenderTexture textureB)
        {
            Vector4 mu1 = Vector4.zero;
            Vector4 mu2 = Vector4.zero;

            int textureW = 16;
            int textureH = 16;
            RenderTexture.active = textureA;
            Texture2D image1 = new (textureW, textureH);
            image1.ReadPixels(new Rect(0, 0, textureA.width, textureA.height), 0, 0);

            RenderTexture.active = textureB;
            Texture2D image2 = new(textureW, textureH);
            image2.ReadPixels(new Rect(0, 0, textureB.width, textureB.height), 0, 0);

            int validPixels = 0;
            for (int w = 0; w < textureW; ++w)
            {
                for(int h = 0; h < textureH; ++h)
                {
                    Vector4 color1 = image1.GetPixel(w, h);
                    Vector4 color2 = image2.GetPixel(w, h);

                    //Debug.Log(color1);
                    //Debug.Log(color2);

                    // If either pixel is transparent
                    if (Vector4.Magnitude(color1) < Util.EPSILON || Vector4.Magnitude(color2) < Util.EPSILON)
                        continue;

                    mu1 += color1;
                    mu2 += color2;
                    validPixels++;
                }
            }
            //Debug.Log("Result");
            //Debug.Log(mu1);
            //Debug.Log(mu2);
            //Debug.Log(validPixels);
            //Debug.Log(mu1 + mu2);

            if (validPixels == 0)
                return Vector4.zero;

            mu1 /= validPixels;
            mu2 /= validPixels;

            //Debug.Log(mu1 + mu2);
            RenderTexture.active = null;
            return mu1 + 4 * mu2;
        }
    }
}

