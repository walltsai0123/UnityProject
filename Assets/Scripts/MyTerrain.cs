using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Mathematics;
using System;

namespace XPBD
{
    //[ExecuteInEditMode]
    [RequireComponent(typeof(Terrain))]
    public class MyTerrain : MonoBehaviour
    {
        public Terrain Terrain { get; private set; }

        public TerrainData TerrainData { get; private set; }
        //[SerializeField] Shader shader;
        public Material CollisionMaterial => collisionMaterial;
        [SerializeField] Material collisionMaterial;

        //[SerializeField] Mesh mesh;
        public Mesh mesh2;
        [SerializeField] bool instancing = true;

        RenderParams renderParams;

        readonly int patchSize = 32;
        int patchX;
        int patchY;

        public void SetupMaterial(MyCollision collision)
        {
            Vector3 tangent = (float3)collision.T;
            Vector3 bitangent = (float3)collision.B;
            collisionMaterial.SetVector("_TangentVector", tangent);
            collisionMaterial.SetVector("_BitangentVector", bitangent);

            float3 terrainPos = transform.position;

            int X = (int)(patchX * (float)(collision.q.x - terrainPos.x) / TerrainData.size.x);
            int Z = (int)(patchY * (float)(collision.q.z - terrainPos.z) / TerrainData.size.z);

            collisionMaterial.SetVector("_TerrainPatchInstanceData", new Vector3(X * 32, Z * 32, 1));
            //Graphics.RenderMesh(new RenderParams(collisionMaterial), mesh2, 0, transform.localToWorldMatrix);
            //Graphics.DrawMesh(mesh2, transform.localToWorldMatrix, collisionMaterial, 0);
            //Debug.Log($"{X} {Z}");
        }

        private void Awake()
        {
            Terrain = GetComponent<Terrain>();
            Initialized();
            InitMaterial();

            Debug.Log(TerrainData.detailPatchCount);
        }

        private void OnEnable()
        {
            Simulation.get.terrain = this;
        }

        private void Update()
        {
        }

        private void Initialized()
        {
            Terrain.drawInstanced = true;
            TerrainData = Terrain.terrainData;
            int heightmapResolution = TerrainData.heightmapResolution;
            //float[,] heights = TerrainData.GetHeights(0, 0, heightmapResolution, heightmapResolution);

            //mesh = new()
            //{
            //    name = "terrainMesh",
            //    indexFormat = IndexFormat.UInt32
            //};

            List<Vector3> vertices = new();
            List<int> indices = new();
            List<Vector2> uv = new();

            //int size = heightmapResolution - 1;
            //float xSize = TerrainData.size.x;
            //float zSize = TerrainData.size.z;

            //for (int y = 0; y <= size; y++)
            //{
            //    for (int x = 0; x <= size; x++)
            //    {
            //        float height = heights[y, x] * TerrainData.size.y;
            //        vertices.Add(new Vector3(x * xSize / size, height, y * zSize / size));
            //        uv.Add(new Vector2((float)x / size, (float)y / size));

            //        if (x < size && y < size)
            //        {
            //            int baseIndex = vertices.Count - 1;
            //            int Size = size + 1;
            //            indices.Add(baseIndex);
            //            indices.Add(baseIndex + Size + 1);
            //            indices.Add(baseIndex + 1);

            //            indices.Add(baseIndex);
            //            indices.Add(baseIndex + Size);
            //            indices.Add(baseIndex + Size + 1);
            //        }
            //    }
            //}
            //mesh.SetVertices(vertices);
            //mesh.SetTriangles(indices, 0);
            //mesh.SetUVs(0, uv);
            //mesh.RecalculateNormals();
            //mesh.RecalculateTangents();

            mesh2 = new Mesh()
            {
                name = "terrainMesh2"
            };

            //patchSize = 32;
            vertices.Clear();
            indices.Clear();
            uv.Clear();
            for (int y = 0; y <= patchSize; y++)
            {
                for (int x = 0; x <= patchSize; x++)
                {
                    vertices.Add(new Vector3(x, y));
                    uv.Add(new Vector2((float)x / patchSize, (float)y / patchSize));

                    if (x < patchSize && y < patchSize)
                    {
                        int baseIndex = vertices.Count - 1;
                        int Size = patchSize + 1;
                        indices.Add(baseIndex);
                        indices.Add(baseIndex + Size + 1);
                        indices.Add(baseIndex + 1);

                        indices.Add(baseIndex);
                        indices.Add(baseIndex + Size);
                        indices.Add(baseIndex + Size + 1);
                    }
                }
            }

            mesh2.SetVertices(vertices);
            mesh2.SetTriangles(indices, 0);
            mesh2.SetUVs(0, uv);
            mesh2.RecalculateNormals();
            //mesh2.RecalculateTangents();
            //mesh2.bounds = new Bounds(Terrain.terrainData.size / 2, Terrain.terrainData.size);

            patchX = (int)(heightmapResolution - 1) / patchSize;
            patchY = (int)(heightmapResolution - 1) / patchSize;
        }

        private void InitMaterial()
        {
            collisionMaterial.enableInstancing = true;
            collisionMaterial.EnableKeyword("_NORMALMAP");

            var terrainLayers = Terrain.terrainData.terrainLayers;

            collisionMaterial.SetTexture($"_Control", Terrain.terrainData.alphamapTextures[0]);

            for (int i = 0; i < terrainLayers.Length; i++)
            {
                TerrainLayer layer = terrainLayers[i];
                collisionMaterial.SetTexture($"_Splat{i}", layer.diffuseTexture);
                collisionMaterial.SetTexture($"_Normal{i}", layer.normalMapTexture);
                collisionMaterial.SetFloat($"_NormalScale{i}", layer.normalScale);
                collisionMaterial.SetTexture($"_Mask{i}", layer.maskMapTexture);

                Vector4 ST = new(1000f / layer.tileSize.x, 1000f / layer.tileSize.y, layer.tileOffset.x, layer.tileOffset.y);
                collisionMaterial.SetVector($"_Splat{i}_ST", ST);
            }

            collisionMaterial.SetTexture("_TerrainHeightmapTexture", Terrain.terrainData.heightmapTexture);
            collisionMaterial.SetTexture("_TerrainNormalmapTexture", Terrain.normalmapTexture);


            int width = Terrain.terrainData.heightmapTexture.width;
            int height = Terrain.terrainData.heightmapTexture.height;

            Vector4 recipSize = new(1.0f / width, 1.0f / height, 1.0f / (width - 1), 1.0f / (height - 1));
            collisionMaterial.SetVector("_TerrainHeightmapRecipSize", recipSize);

            Vector3 heightmapScale = Terrain.terrainData.heightmapScale;
            heightmapScale.y *= (65535f / 32766.0f);
            collisionMaterial.SetVector("_TerrainHeightmapScale", heightmapScale);

            collisionMaterial.SetVector("_TerrainPatchInstanceData", new Vector3(0, 0, 1));
        }

        void RenderTerrainMesh(Mesh terrainMesh, Material terrainMaterial, Matrix4x4 transform)
        {
            renderParams = new(terrainMaterial);
            renderParams.worldBounds = new Bounds(Terrain.terrainData.size / 2, Terrain.terrainData.size);
            renderParams.material.EnableKeyword("_NORMALMAP");
            renderParams.material.enableInstancing = instancing;
            var terrainLayers = Terrain.terrainData.terrainLayers;

            MaterialPropertyBlock materialPropertyBlock = new();
            renderParams.matProps = materialPropertyBlock;

            Texture2D[] controlTextures = Terrain.terrainData.alphamapTextures;
            materialPropertyBlock.SetTexture($"_Control", controlTextures[0]);

            for (int i = 0; i < terrainLayers.Length; i++)
            {
                TerrainLayer layer = terrainLayers[i];
                materialPropertyBlock.SetTexture($"_Splat{i}", layer.diffuseTexture);
                materialPropertyBlock.SetTexture($"_Normal{i}", layer.normalMapTexture);
                materialPropertyBlock.SetFloat($"_NormalScale{i}", layer.normalScale);
                //materialPropertyBlock.SetTexture($"_Mask{i}", layer.maskMapTexture);

                Vector4 ST = new(1000f / layer.tileSize.x, 1000f / layer.tileSize.y, layer.tileOffset.x, layer.tileOffset.y);
                materialPropertyBlock.SetVector($"_Splat{i}_ST", ST);
                //materialPropertyBlock.SetVector($"_Splat_{i}", layer.tileOffset);
            }

            if (instancing)
            {
                Terrain.drawInstanced = true;
                materialPropertyBlock.SetTexture("_TerrainHeightmapTexture", Terrain.terrainData.heightmapTexture);
                materialPropertyBlock.SetTexture("_TerrainNormalmapTexture", Terrain.normalmapTexture);


                int width = Terrain.terrainData.heightmapTexture.width;
                int height = Terrain.terrainData.heightmapTexture.height;

                Vector4 recipSize = new(1.0f / width, 1.0f / height, 1.0f / (width - 1), 1.0f / (height - 1));
                materialPropertyBlock.SetVector("_TerrainHeightmapRecipSize", recipSize);

                Vector3 heightmapScale = Terrain.terrainData.heightmapScale;
                heightmapScale.y *= (65535f / 32766.0f);
                materialPropertyBlock.SetVector("_TerrainHeightmapScale", heightmapScale);

                materialPropertyBlock.SetVector("_TerrainPatchInstanceData", new Vector3(0, 0, 1));
                //Graphics.RenderMesh(renderParams, mesh2, 0, transform);

                for (int i = 0; i < patchX; i++)
                {
                    for (int j = 0; j < patchY; j++)
                    {
                        materialPropertyBlock.SetVector("_TerrainPatchInstanceData", new Vector3(i * 32, j * 32, 1));
                        Graphics.RenderMesh(renderParams, mesh2, 0, transform);
                    }
                }
            }
            else
            {
                Graphics.RenderMesh(renderParams, terrainMesh, 0, transform);
            }
        }

    }
}