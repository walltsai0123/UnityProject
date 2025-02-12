using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Rendering;
using System.Net;
using System.Runtime.CompilerServices;
using UnityEngine.XR;
using static UnityEngine.ParticleSystem;






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
    //[RequireComponent(typeof(MeshRenderer), typeof(MeshFilter))]
    public class TerrainSystem : MonoBehaviour
    {
        [SerializeField] Texture2D inputTerrain;
        [SerializeField] Material terrainMat;
        [SerializeField] Material sandMat;
        [SerializeField] HeightMapMesh terrain;
        [SerializeField] HeightMapMesh sandSurface;

        [Header("Terrain Settings")]
        [SerializeField, Min(1)] int gridSize = 1024;
        [SerializeField, Min(1)] float TerrainWidth;
        [SerializeField, Min(0)] float maxHeight = 200f;
        [SerializeField, Min(0)] float sandRegion = 1;
        [SerializeField] float depth = 1;
        [SerializeField, Range(0,1)] float mu;
        int numCells;

        RenderTexture terrainHeightMap;
        RenderTexture terrainNormalMap;
        RenderTexture sandHeightMap;
        RenderTexture sandNormalMap;

        REAL[] terrainHeights;
        REAL[] heights;
        REAL[] u;
        REAL[] v;
        float spacing;
        readonly float gravity = -9.81f;

        // Compute buffers
        [SerializeField] ComputeShader computeShader;
        ComputeBuffer terrainHeightBuffer;
        ComputeBuffer heightBuffer;
        ComputeBuffer heightCacheBuffer;
        ComputeBuffer uBuffer;
        ComputeBuffer vBuffer;
        ComputeBuffer contactBuffer;

        readonly int heightKernel = 0;
        readonly int cacheKernel = 1;
        readonly int velocityKernel = 2;
        readonly int frictionKernel = 3;
        readonly int textureKernel = 4;
        readonly int collisionKernel = 5;


        [Header("Gaussian Filter")]
        [SerializeField]
        bool filter = true;
        [SerializeField]
        Shader gaussianShader;
        [SerializeField]
        RenderTexture gaussianTexture;
        private Material gaussianMaterial;

        [Header("Height To Normal Filter")]
        [SerializeField]
        Shader H2NShader;
        private Material H2N_Material;
        [SerializeField, Min(0)]
        float Strength;

        // GPU Readback
        Timer timer = new Timer();
        bool isReadbackRequested = false;
        AsyncGPUReadbackRequest heightRequest;
        AsyncGPUReadbackRequest uRequest;
        AsyncGPUReadbackRequest vRequest;
        bool hasStarted = false;

        public bool IsInside(REAL3 position)
        {
            REAL3 relPos = position - (float3)transform.position;
            bool xInside = 0 <= relPos.x && relPos.x <= TerrainWidth;
            bool yInside = 0 <= relPos.y && relPos.y <= maxHeight || true;
            bool zInside = 0 <= relPos.z && relPos.z <= TerrainWidth;

            return xInside && yInside && zInside;
        }

        public int2 GetGrid(REAL3 position)
        {
            REAL x = (position.x - transform.position.x) / spacing;
            REAL z = (position.z - transform.position.z) / spacing;

            int gridX = (int)math.floor(x);
            int gridZ = (int)math.floor(z);

            return new int2(gridX, gridZ);
        }
        public int GridToIndex(int2 grid)
        {
            return grid.x * gridSize + grid.y;
        }
        public REAL TerrainHeight(int x, int z)
        {
            //Check coordinate within range
            if (x < 0 || x >= gridSize || z < 0 || z >= gridSize)
                return 0;

            int index = x * gridSize + z;
            REAL result = terrainHeights[index];
            return result;
        }
        public REAL Height(int x, int z)
        {
            //Check coordinate within range
            if (x < 0 || x >= gridSize || z < 0 || z >= gridSize)
                return 0;

            int index = x * gridSize + z;
            REAL result = heights[index] + terrainHeights[index];
            return result;
        }
        public REAL SampleTerrainHeight(REAL3 position)
        {
            REAL x = (position.x - transform.position.x) / spacing;
            REAL z = (position.z - transform.position.z) / spacing;

            int gridX = (int)math.floor(x);
            int gridZ = (int)math.floor(z);
            if (gridX < 0 || gridX >= gridSize || gridZ < 0 || gridZ >= gridSize)
                return REAL.MinValue;

            REAL h00 = TerrainHeight(gridX, gridZ);
            REAL h10 = TerrainHeight(gridX + 1, gridZ);

            REAL h01 = TerrainHeight(gridX, gridZ + 1);
            REAL h11 = TerrainHeight(gridX + 1, gridZ + 1);
            REAL2 cellUV = new(x - gridX, z - gridZ);

            REAL h0 = math.lerp(h00, h10, cellUV.x);
            REAL h1 = math.lerp(h01, h11, cellUV.x);
            return math.lerp(h0, h1, cellUV.y);
        }
        public REAL SampleHeight(REAL3 position)
        {
            CheckReadbackStatus();

            REAL x = (position.x - transform.position.x) / spacing;
            REAL z = (position.z - transform.position.z) / spacing;

            int gridX = (int)math.floor(x);
            int gridZ = (int)math.floor(z);

            if (gridX < 0 || gridX >= gridSize || gridZ < 0 || gridZ >= gridSize)
                return REAL.MinValue;

            REAL h00 = Height(gridX, gridZ);
            REAL h10 = Height(gridX + 1, gridZ);

            REAL h01 = Height(gridX, gridZ + 1);
            REAL h11 = Height(gridX + 1, gridZ + 1);
            REAL2 cellUV = new(x - gridX, z - gridZ);

            REAL h0 = math.lerp(h00, h10, cellUV.x);
            REAL h1 = math.lerp(h01, h11, cellUV.x);
            return math.lerp(h0, h1, cellUV.y);
        }

        public REAL3 SampleTerrainNormal(REAL3 position) {
            REAL delta = 0.5;
            REAL h00 = SampleTerrainHeight(position - delta * new REAL3(1, 0, 0));
            REAL h01 = SampleTerrainHeight(position + delta * new REAL3(1, 0, 0));
            REAL h10 = SampleTerrainHeight(position - delta * new REAL3(0, 0, 1));
            REAL h11 = SampleTerrainHeight(position + delta * new REAL3(0, 0, 1));

            REAL3 normal;
            normal.x = (h00 - h01);
            normal.y = 2 * delta;
            normal.z = (h10 - h11);
            return math.normalize(normal);
        }
        public REAL3 SampleNormal(REAL3 position)
        {
            CheckReadbackStatus();

            REAL delta = 0.5;
            REAL h00 = SampleHeight(position - delta * new REAL3(1, 0, 0));
            REAL h01 = SampleHeight(position + delta * new REAL3(1, 0, 0));
            REAL h10 = SampleHeight(position - delta * new REAL3(0, 0, 1));
            REAL h11 = SampleHeight(position + delta * new REAL3(0, 0, 1));

            REAL3 normal;
            normal.x = (h00 - h01);
            normal.y = 2 * delta;
            normal.z = (h10 - h11); 
            return math.normalize(normal);
        }

        public void Simulate(REAL dt, REAL3 gravity)
        {
            UpdateGPUSettings(dt, gravity);
            AdvectionGPU();
            UpdateVelocityGPU();
        }

        public void EndFrame()
        {
            UpdateTextureGPU();
            RequestReadback();
        }

        public void FluidToSolid(SoftBodySystem softBodySystem, REAL dt)
        {
            if (softBodySystem.VerticesNum == 0)
                return;

            CheckReadbackStatus();

            for(int i = 0; i < softBodySystem.FacesNum; ++i)
            {
                // Extract particles from faces
                int3 face = softBodySystem.surfaceFaces[i];
                SoftBodyParticle particle1 = softBodySystem.particles[face[0]];
                SoftBodyParticle particle2 = softBodySystem.particles[face[1]];
                SoftBodyParticle particle3 = softBodySystem.particles[face[2]];

                // Face total mass and weights
                REAL M = 1 / particle1.invMass + 1 / particle2.invMass + 1 / particle3.invMass;
                REAL3 weight = new();
                weight[0] = 1 / (particle1.invMass * M);
                weight[1] = 1 / (particle2.invMass * M);
                weight[2] = 1 / (particle3.invMass * M);

                // centroid position and velocity
                REAL3 pos = weight[0] * particle1.pos + weight[1] * particle2.pos + weight[2] * particle3.pos;
                REAL3 vel = weight[0] * particle1.vel + weight[1] * particle2.vel + weight[2] * particle3.vel;

                // skip if not inside terrain
                if (!IsInside(pos)) continue;

                // get grid index
                int2 grid = GetGrid(pos);
                int index = GridToIndex(grid);

                // Skip if sand/fluid height too small 
                if (heights[index] < Util.EPSILON)
                    continue;

                // Get penetration depth
                REAL depth = math.min(heights[index], heights[index] + terrainHeights[index] - pos.y);

                // Skip if not penetrate
                if (depth < 0)
                    continue;

                // Sand density kg/m^3
                REAL phi = 1600;

                // Get particle area
                //REAL V_disp = particle.volume;
                REAL A_disp = 0.5 * math.length(math.cross(particle2.pos - particle1.pos, particle3.pos - particle1.pos));

                REAL f_n = -phi * Simulation.get.gravity.y * A_disp * depth;
                REAL3 F_n = f_n * new REAL3(0, 1, 0);

                // V_rel = V_solid - V_fluid
                REAL3 V_rel = vel - new REAL3(U(grid.x, grid.y), 0, V(grid.x, grid.y));
                V_rel.y = 0;

                REAL3 F_t = -mu * math.abs(F_n.y) * math.normalizesafe(V_rel, 0);
                REAL3 F_ext = F_n + F_t;

                particle1.f_ext += F_ext * weight[0];
                particle2.f_ext += F_ext * weight[1];
                particle3.f_ext += F_ext * weight[2];

                softBodySystem.particles[face[0]] = particle1;
                softBodySystem.particles[face[1]] = particle2;
                softBodySystem.particles[face[2]] = particle3;
            }

            for (int i = 0; i < softBodySystem.VerticesNum; ++i)
            {
                break;
                // Get particle position
                SoftBodyParticle particle = softBodySystem.particles[i];
                REAL3 pos = particle.pos;

                // skip if not inside terrain
                if (!IsInside(pos)) continue;

                // get grid index
                int2 grid = GetGrid(pos);
                int index = GridToIndex(grid);

                // Skip if sand/fluid height too small 
                if (heights[index] < Util.EPSILON)
                    continue;

                // Get penetration depth
                REAL depth = math.min(heights[index], heights[index] + terrainHeights[index] - pos.y);

                // Skip if not penetrate
                if (depth < 0)
                    continue;

                // Sand density kg/m^3
                REAL phi = 1602;

                // Get particle area
                REAL V_disp = particle.volume;
                REAL A_disp = math.min(spacing * spacing, math.pow(particle.volume, 2.0 / 3.0));

                REAL3 F_n = (particle.vel.y >= 0) ? 0 : -phi * Simulation.get.gravity * A_disp * depth;

                // V_rel = V_solid - V_fluid
                REAL3 V_rel = particle.vel - new REAL3(U(grid.x, grid.y), 0, V(grid.x, grid.y));
                //V_rel = particle.vel;
                V_rel.y = 0;

                REAL3 F_t = -mu * math.abs(F_n.y) * math.normalizesafe(V_rel, 0);

                particle.f_ext = F_n + F_t;

                softBodySystem.particles[i] = particle;
            }
            
        }
        REAL U(int i, int j)
        {
            if (i <= 0 || i >= gridSize || j <= 0 || j >= gridSize) return 0;

            int index = GridToIndex(new int2(i, j));
            return u[index];
        }
        REAL V(int i, int j)
        {
            if (i <= 0 || i >= gridSize || j <= 0 || j >= gridSize) return 0;

            int index = GridToIndex(new int2(i, j));
            return v[index];
        }
        public void SolidToFluid(SoftBodySystem softBodySystem, REAL dt)
        {
            if (softBodySystem.VerticesNum == 0)
                return;

            CheckReadbackStatus();

            for (int i = 0; i < softBodySystem.FacesNum; ++i)
            {
                // Extract particles from faces
                int3 face = softBodySystem.surfaceFaces[i];
                SoftBodyParticle particle1 = softBodySystem.particles[face[0]];
                SoftBodyParticle particle2 = softBodySystem.particles[face[1]];
                SoftBodyParticle particle3 = softBodySystem.particles[face[2]];

                // Face total mass and weights
                REAL M = 1 / particle1.invMass + 1 / particle2.invMass + 1 / particle3.invMass;
                REAL3 weight = new();
                weight[0] = 1 / (particle1.invMass * M);
                weight[1] = 1 / (particle2.invMass * M);
                weight[2] = 1 / (particle3.invMass * M);

                // centroid position and velocity
                REAL3 pos = weight[0] * particle1.prevPos + weight[1] * particle2.prevPos + weight[2] * particle3.prevPos;
                REAL3 vel = weight[0] * particle1.vel + weight[1] * particle2.vel + weight[2] * particle3.vel;

                // skip if not inside terrain
                if (!IsInside(pos)) continue;

                // get grid index
                int2 grid = GetGrid(pos);
                int index = GridToIndex(grid);

                // Get particle and fluid/sand volume
                REAL3 v_fluid = new REAL3(U(grid.x, grid.y), 0, V(grid.x, grid.y));
                REAL3 V_rel = vel - v_fluid;
                REAL3 N = math.cross(particle2.pos - particle1.pos, particle3.pos - particle1.pos);
                REAL3 n = math.normalize(N);
                REAL A = 0.5 * math.length(N);
                REAL V_disp = math.abs(math.dot(n, V_rel) * A * dt);
                int sign = (int)math.sign(n.y);

                int passgrids = (int)math.floor(math.length(vel - vel.y * new REAL3(0, 1, 0)) * dt / spacing + 0.5);
                int substeps = math.max(1, passgrids);

                
                for(int q = 1; q <= substeps; q++)
                {
                    // point on path
                    REAL3 p_s = pos + vel * dt * q / substeps;

                    int2 grid_s = GetGrid(p_s);
                    int index_s = GridToIndex(grid_s);

                    // penetrate depth
                    REAL depth_s = math.min(heights[index_s], heights[index_s] + terrainHeights[index_s] - p_s.y);

                    // fluid surface level
                    REAL level = heights[index_s] + terrainHeights[index_s];

                    // Skip if not penetrate
                    if (depth_s <= 0)
                        continue;

                    REAL decay = math.exp(-depth_s); decay = 1;
                    REAL h_delta = V_disp / (substeps * spacing * spacing);
                    heights[index_s] -= h_delta;

                    REAL coeff = math.min(1, 0.2 * depth_s * A * dt / (level * spacing * spacing));

                    // Check neighbor cell type (solid/fluid)
                    int[] S = new int[4];
                    S[0] = (grid_s.x == gridSize - 1) ? 0 : 1;
                    S[1] = (grid_s.x == 0) ? 0 : 1;
                    S[2] = (grid_s.y == gridSize - 1) ? 0 : 1;
                    S[3] = (grid_s.y == 0) ? 0 : 1;

                    REAL u_delta = V_rel.x / (S[0] + S[1]);
                    REAL v_delta = V_rel.z / (S[2] + S[3]);
                    REAL w_delta = math.min(0, -h_delta) / (S[0] + S[1] + S[2] + S[3]);
                    w_delta = 0;

                    int topIndex = GridToIndex(grid + new int2(0, 1));
                    int rightIndex = GridToIndex(grid + new int2(1, 0));
                    int bottomIndex = index;
                    int leftIndex = index;

                    REAL maxVel = 0.5 * spacing / dt;
                    // Change face velocities
                    if (S[0] == 1)
                    {
                        //u[rightIndex] += coeff * (particle.vel.x - u[rightIndex]);
                        u[rightIndex] += coeff * (u_delta - w_delta);
                        u[rightIndex] = math.clamp(u[rightIndex], -maxVel, maxVel);
                    }
                    if (S[1] == 1)
                    {
                        u[leftIndex] += coeff * (u_delta + w_delta);
                        u[leftIndex] = math.clamp(u[leftIndex], -maxVel, maxVel);
                    }
                    if (S[2] == 1)
                    {
                        //v[topIndex] += coeff * (particle.vel.z - v[topIndex]);
                        v[topIndex] += coeff * (v_delta - w_delta);
                        v[topIndex] = math.clamp(v[topIndex], -maxVel, maxVel);
                    }
                    if (S[3] == 1)
                    {
                        v[bottomIndex] += coeff * (v_delta + w_delta);
                        v[bottomIndex] = math.clamp(v[bottomIndex], -maxVel, maxVel);
                    }
                }
            }

            heightBuffer.SetData(heights);
            uBuffer.SetData(u);
            vBuffer.SetData(v);
        }

        private void ConstructMesh()
        {
            // Create child heightmesh
            if(terrain == null)
            {
                GameObject gameObject = new GameObject("terrain");
                gameObject.transform.parent = transform;
                terrain = gameObject.AddComponent<HeightMapMesh>();
            }
            if(sandSurface == null)
            {
                GameObject gameObject = new GameObject("sandSurface");
                gameObject.transform.parent = transform;
                sandSurface = gameObject.AddComponent<HeightMapMesh>();
            }
            terrain.Initialize(gridSize, TerrainWidth);
            sandSurface.Initialize(gridSize, TerrainWidth);

            //SetUp filters
            gaussianMaterial = new Material(gaussianShader);
            gaussianTexture = new RenderTexture(gridSize, gridSize, 0, RenderTextureFormat.ARGB32);
            H2N_Material = new Material(H2NShader);
            spacing = TerrainWidth / (gridSize - 1);
            Strength = maxHeight / spacing;
            H2N_Material.SetFloat("_Strength", Strength);

            // Set Terrain Maps
            terrainHeightMap = terrain.Height_Map;
            terrainNormalMap = terrain.Normal_Map;
            Graphics.Blit(inputTerrain, terrainHeightMap);
            //Graphics.CopyTexture(inputTerrain, terrainHeightMap);
            GaussianFilt(terrainHeightMap);
            Graphics.Blit(terrainHeightMap, terrainNormalMap, H2N_Material);
            terrainMat.SetTexture("_ParallaxMap", terrainHeightMap);
            terrainMat.SetTexture("_TerrainNormalMap", terrainNormalMap);
            terrainMat.SetFloat("_Parallax", maxHeight);
            terrain.material = terrainMat;

            // Set Sand Maps
            sandHeightMap = sandSurface.Height_Map;
            sandNormalMap = sandSurface.Normal_Map;
            GaussianFilt(sandHeightMap);
            Graphics.Blit(sandHeightMap, sandNormalMap, H2N_Material);
            sandMat.SetTexture("_ParallaxMap", sandHeightMap);
            sandMat.SetTexture("_TerrainNormalMap", sandNormalMap);
            sandMat.SetFloat("_Parallax", maxHeight);
            sandSurface.material = sandMat;
        }

        private void Start()
        {
            ConstructMesh();
            InitGridData();
            InitComputeBuffers();
        }
        private void OnValidate()
        {
            if (Application.isPlaying)
                return;
            ConstructMesh();
        }

        private void OnDestroy()
        {
            ComputeHelper.Release(terrainHeightBuffer, heightBuffer, heightCacheBuffer, uBuffer, vBuffer, contactBuffer);
        }
        private void InitGridData()
        {
            numCells = gridSize * gridSize;

            terrainHeights = new REAL[numCells];
            heights = new REAL[numCells];
            u = new REAL[numCells];
            v = new REAL[numCells];

            //Get terrain heights from rendertexture
            Color[] heightColor = GetRenderTexturePixels(terrainHeightMap);

            for(int i = 0; i < heightColor.Length; i++)
                terrainHeights[i] = heightColor[i].r * maxHeight;

            //System.Array.Fill(terrainHeights, 0.00f);
            System.Array.Fill(heights, 0.00f);
            for (int i = 0; i < gridSize; i++)
            {
                for (int j = 0; j < gridSize; j++)
                {
                    int id = i * gridSize + j;
                    if (terrainHeights[id] < sandRegion)
                        heights[id] = sandRegion - terrainHeights[id];
                }
            }

            System.Array.Fill(u, 0f);
            System.Array.Fill(v, 0f);
        }
        private void InitComputeBuffers()
        {
            terrainHeightBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            heightBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            heightCacheBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            uBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            vBuffer = new ComputeBuffer(numCells, sizeof(REAL));

            terrainHeightBuffer.SetData(terrainHeights);
            heightBuffer.SetData(heights);
            uBuffer.SetData(u);
            vBuffer.SetData(v);

            ComputeHelper.SetBuffer(computeShader, terrainHeightBuffer, "terrainHeights", velocityKernel, textureKernel);
            ComputeHelper.SetBuffer(computeShader, heightBuffer, "heights", heightKernel, cacheKernel, velocityKernel, textureKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, heightCacheBuffer, "heights_cache", heightKernel, cacheKernel);

            ComputeHelper.SetBuffer(computeShader, uBuffer, "u", heightKernel, velocityKernel, frictionKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, vBuffer, "v", heightKernel, velocityKernel, frictionKernel, collisionKernel);

            ComputeHelper.AssignTexture(computeShader, sandHeightMap, "sandHeightMap", textureKernel);
            ComputeHelper.AssignTexture(computeShader, sandNormalMap, "sandNormalMap", textureKernel);

            ComputeHelper.AssignTexture(computeShader, terrainHeightMap, "terrainHeightMap", heightKernel, velocityKernel, textureKernel);
            ComputeHelper.AssignTexture(computeShader, terrainNormalMap, "terrainNormalMap", textureKernel);

            // Set const
            spacing = TerrainWidth / (gridSize - 1);
            computeShader.SetFloat("spacing", (float)spacing);
            computeShader.SetFloat("gravity", (float)gravity);
            computeShader.SetInt("numX", gridSize);
            computeShader.SetInt("numZ", gridSize);
            computeShader.SetFloat("maxHeight", (float)maxHeight);
        }

        private void UpdateGPUSettings(REAL dt, REAL3 gravity)
        {
            computeShader.SetFloat("dt", (float)dt);
            computeShader.SetFloat("gravity", (float)gravity.y);
            computeShader.SetFloat("mu", (float)mu);
        }
        private void AdvectionGPU()
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: heightKernel);
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: cacheKernel);
        }

        private void UpdateVelocityGPU()
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: velocityKernel);
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: frictionKernel);
        }

        void UpdateTextureGPU()
        {
            ComputeHelper.Dispatch(computeShader, sandHeightMap, textureKernel);
            GaussianFilt(sandHeightMap);
            Graphics.Blit(terrainHeightMap, terrainNormalMap, H2N_Material);
            Graphics.Blit(sandHeightMap, sandNormalMap, H2N_Material);
        }
        void RequestReadback()
        {
            timer.Tic();
            heightRequest = AsyncGPUReadback.Request(heightBuffer);
            uRequest = AsyncGPUReadback.Request(uBuffer);
            vRequest = AsyncGPUReadback.Request(vBuffer);
            isReadbackRequested = true;
        }

        void CheckReadbackStatus()
        {
            if (!isReadbackRequested)
            {
                //Debug.Log("No readback request has been made.");
                return;
            }

            while (!heightRequest.done)
            {
                heightRequest.WaitForCompletion();
            }
            while (!uRequest.done)
            {
                uRequest.WaitForCompletion();
            }
            while (!vRequest.done)
            {
                vRequest.WaitForCompletion();
            }

            if (heightRequest.done && uRequest.done && vRequest.done)
            {
                if (heightRequest.hasError)
                {
                    Debug.LogError("GPU Readback Error!");
                }
                else
                {
                    heights = heightRequest.GetData<REAL>().ToArray();
                    u = uRequest.GetData<REAL>().ToArray();
                    v = vRequest.GetData<REAL>().ToArray();
                }

                isReadbackRequested = false;
            }
        }
        Color[] GetRenderTexturePixels(RenderTexture renderTexture)
        {
            RenderTexture.active = renderTexture;

            Texture2D tempTexture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);

            tempTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
            tempTexture.Apply();

            // 獲取像素數據
            Color[] widthMajor = tempTexture.GetPixels();
            Object.Destroy(tempTexture);

            int width = renderTexture.width;
            int height = renderTexture.height;

            Color[] heightMajor = new Color[widthMajor.Length];
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    heightMajor[i * height + j] = widthMajor[i + j * width];
                }
            }

            return heightMajor;
        }

        private void GaussianFilt(RenderTexture renderTexture)
        {
            if(!filter)
                return;
            Graphics.Blit(renderTexture, gaussianTexture, gaussianMaterial);
            Graphics.Blit(gaussianTexture, renderTexture);
        }
    }
}

