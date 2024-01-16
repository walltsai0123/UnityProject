using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace XPBD
{
    public class CollisionDetect
    {
        static CollisionCamera cam1;
        static CollisionCamera cam2;

        static CollisionDetect()
        {
            GameObject[] gameObjects = GameObject.FindGameObjectsWithTag("CollisionCam");
            if (gameObjects.Length != 2)
                Debug.LogWarning("Collision camera should only be two!!! Current: " + gameObjects.Length);
            cam1 = gameObjects[0].GetComponent<CollisionCamera>();
            cam2 = gameObjects[1].GetComponent<CollisionCamera>();

            Debug.Log(cam1);
            Debug.Log(cam2);
        }
        public static List<CollisionConstraint> CollectCollision(Body body, Primitive primitive, float dt)
        {
            if (body.bodyType == Body.BodyType.Soft)
                return SoftBodyCollision((SoftBody)body, primitive, dt);
            return new List<CollisionConstraint>();
        }

        public static List<CollisionConstraint> RigidCollision(Rigid rigid)
        {
            List<CollisionConstraint> newList = new(rigid.rigidCollisions);
            return newList;
        }

        private static List<CollisionConstraint> SoftBodyCollision(SoftBody soft, Primitive primitive, float dt)
        {
            Bounds bounds = primitive.collider.bounds;
            List<CollisionConstraint> newList = new();

            for (int i = 0; i < soft.VerticesNum; ++i)
            {
                float3 predPos = soft.Pos[i] + soft.vel[i] * dt;
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
                    newList.Add(newCollision);

                    float3 N = newCollision.N;
                    float3 up = math.cross(newCollision.N, new float3(1, 0, 0));
                    if (math.length(up) < 1e-6)
                    {
                        up = -math.cross(newCollision.N, new float3(0, 0, -1));
                    }
                    math.normalize(up);

                    cam1.transform.position = soft.Pos[i] - 0.01f * N;
                    cam2.transform.position = newCollision.q + 0.01f * N;

                    cam1.transform.LookAt(cam2.transform, up);
                    cam2.transform.LookAt(cam1.transform, up);
                }
            }

            return newList;
        }
    }
}

