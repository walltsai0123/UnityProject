using UnityEngine;
using Unity.Mathematics;

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
    public class SandSurface : Primitive
    {
        public XPBD.Plane Plane { get; private set; }

        [SerializeField] REAL sizeX;
        [SerializeField] REAL sizeZ;
        [SerializeField] REAL spacing;
        [SerializeField] REAL depth;
        [SerializeField] REAL dilute;
        [SerializeField] REAL ground_elavation;
        [SerializeField] REAL mu;
        [SerializeField] Material meshMaterial;
        [SerializeField] ComputeShader computeShader;
        [SerializeField] private bool updateMesh = false;

        int numX;
        int numZ;
        int numCells;

        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        //Mesh mesh;
        private Vector3[] meshVertices;

        REAL[] heights;
        REAL[] u;
        REAL[] v;
        REAL totalHeight = 0f;

        readonly REAL gravity = -9.81f;

        //Material material;
        RenderTexture height_map;
        RenderTexture normal_map;
        
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

        struct Contact
        {
            public int index;
            public REAL3 point;
            public REAL3 velocity;
            public REAL penetration;
        };

        protected override void Awake()
        {
            base.Awake();
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();
        }

        protected override void Start()
        {
            base.Start();

            Initialize();
            GenerateMesh();
            GenerateTexture();

            InitComputeBuffers();
        }

        private void OnDestroy()
        {
            ComputeHelper.Release(heightBuffer, heightCacheBuffer, uBuffer, vBuffer, contactBuffer);
        }
        public override void Simulate(REAL dt)
        {
            Timer timer = new Timer();
            timer.Tic();
            UpdateGPUSettings(dt);
            AdvectionGPU();
            UpdateVelocityGPU();
            timer.Toc();  

            //timer.Report("Time step: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
        }

        public override void UpdateVisual()
        {
            UpdateVisMesh();
            UpdateTextureGPU();
        }

        public override void ApplyVelocity(REAL dt)
        {
            //if(collisions.Count == 0)
            //    return;

            //Contact [] contact = new Contact[collisions.Count];

            //Vector3 O = meshVertices[0];
            //REAL maxDist = 0f;
            //for (int i = 0; i < collisions.Count; i++)
            //{
            //    var collision = collisions[i];
            //    Vector3 localPoint = transform.InverseTransformPoint((float3)collision.q);
            //    Vector3 localVel = transform.InverseTransformDirection((float3)collision.body.Vel[collision.index]);

            //    REAL offset = spacing / 2f;
            //    int X = (int)math.floor((localPoint.x - O.x + offset) / spacing)/* + Mathf.FloorToInt(numX / 2)*/;
            //    int Z = (int)math.floor((localPoint.z - O.z + offset) / spacing)/* + Mathf.FloorToInt(numZ / 2)*/;

            //    Vector3 V = meshVertices[X * numZ + Z];
            //    V.y = 0f;
            //    REAL dist = (localPoint - V).magnitude;
            //    maxDist = math.max(maxDist, dist);
            //    if (dist > spacing)
            //    {
            //        Debug.Log("pos: " + V);
            //        Debug.Log("localPoint: " + localPoint);
            //        Debug.Log("dist: " + dist);
            //    }

            //    REAL pene = math.length(collision.q - collision.body.Pos[collision.index]);
            //    contact[i] = new Contact()
            //    {
            //        index = X * numZ + Z,
            //        point = (float3)localPoint,
            //        velocity = (float3)localVel,
            //        penetration = pene
            //    };
            //}
            //computeShader.SetFloat("dt", (float)dt);
            //contactBuffer.SetData(contact);
            //ComputeHelper.Dispatch(computeShader, collisions.Count, kernelIndex: collisionKernel);
        }

        private void Initialize()
        {
            numX = (int)math.floor(sizeX / spacing) + 1;
            numZ = (int)math.floor(sizeZ / spacing) + 1;

            numCells = numX * numZ;

            heights = new REAL[numCells];
            u = new REAL[numCells];
            v = new REAL[numCells];
            totalHeight = 0;

            System.Array.Fill(heights, 0.01f);
            System.Array.Fill(u, 0f);
            System.Array.Fill(v, 0f);

            int dimX = Mathf.Min(30, numX / 4);
            int dimZ = Mathf.Min(30, numZ / 4);
            for (int i = 0; i < numX; i++)
            {
                for (int j = 0; j < numZ; j++)
                {
                    int id = i * numZ + j;
                    heights[id] = UnityEngine.Random.Range(0.01f, (float)depth);
                }
            }
            
            for (int i = 0; i < numCells; i++)
            {
                totalHeight += heights[i];
            }
            Plane = GetComponent<XPBD.Plane>();
            Plane.size = new REAL2(sizeX, sizeZ);
        }

        private void GenerateMesh()
        {
            //Generate the mesh's vertices and uvs
            Vector3[] positions = new Vector3[numCells];
            Vector2[] uvs = new Vector2[numCells];

            //Center of the mesh
            int cx = Mathf.FloorToInt(numX / 2f);
            int cz = Mathf.FloorToInt(numZ / 2f);

            for (int i = 0; i < numX; i++)
            {
                for (int j = 0; j < numZ; j++)
                {
                    REAL posX = (i - cx) * spacing;
                    REAL posY = 0f;
                    REAL posZ = (j - cz) * spacing;

                    positions[i * numZ + j] = (float3)new REAL3(posX, posY, posZ);

                    REAL u = i / (REAL)numX;
                    REAL v = j / (REAL)numZ;

                    uvs[i * numZ + j] = (float2)new REAL2(u, v);
                }
            }

            //Build triangles from the vertices
            //If the grid is 3x3 cells (16 vertices) we need a total of 18 triangles
            //-> 18*3 = 54 triangle indices are needed
            //numX is vertices: (4-3)*(4-3)*2*3 = 54
            int[] index = new int[(this.numX - 1) * (this.numZ - 1) * 2 * 3];

            int pos = 0;

            for (int i = 0; i < this.numX - 1; i++)
            {
                for (int j = 0; j < this.numZ - 1; j++)
                {
                    int id0 = i * this.numZ + j;
                    int id1 = i * this.numZ + j + 1;
                    int id2 = (i + 1) * this.numZ + j + 1;
                    int id3 = (i + 1) * this.numZ + j;

                    index[pos++] = id0;
                    index[pos++] = id1;
                    index[pos++] = id2;

                    index[pos++] = id0;
                    index[pos++] = id2;
                    index[pos++] = id3;
                }
            }

            //Generate the mesh itself
            Mesh newMesh = new()
            {
                name = "SandSurface",
            };
            if (numCells > 65535)
                newMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            //To make it faster to update the mesh often
            newMesh.SetVertices(positions);
            newMesh.SetIndices(index, MeshTopology.Triangles, 0);
            newMesh.SetUVs(0, uvs);
            newMesh.MarkDynamic();

            mesh = newMesh;
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();

            meshVertices = positions;

            meshFilter.mesh = mesh;
        }

        private void GenerateTexture()
        {
            material = meshRenderer.material;
            height_map = new RenderTexture(numX, numZ, 32);
            normal_map = new RenderTexture(numX, numZ, 32);

            height_map.enableRandomWrite = true;
            normal_map.enableRandomWrite = true;

            height_map.Create();
            normal_map.Create();

            material.EnableKeyword("_NORMALMAP");
            material.EnableKeyword("_PARALLAXMAP");

            material.SetTexture("_ParallaxMap", height_map);
            material.SetTexture("_BumpMap", normal_map);
        }

        private void InitComputeBuffers()
        {
            heightBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            heightCacheBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            uBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            vBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            contactBuffer = new ComputeBuffer(numCells, System.Runtime.InteropServices.Marshal.SizeOf(typeof(Contact)));

            heightBuffer.SetData(heights);
            uBuffer.SetData(u);
            vBuffer.SetData(v);

//#if USE_FLOAT
//            computeShader.EnableKeyword("USE_FLOAT");
//            computeShader.DisableKeyword("USE_DOUBLE");
//#else
//            computeShader.EnableKeyword("USE_DOUBLE");
//            computeShader.DisableKeyword("USE_FLOAT");
//#endif

            ComputeHelper.SetBuffer(computeShader, heightBuffer, "heights", heightKernel, cacheKernel, velocityKernel, textureKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, heightCacheBuffer, "heights_cache", heightKernel, cacheKernel);

            ComputeHelper.SetBuffer(computeShader, uBuffer, "u", heightKernel, velocityKernel, frictionKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, vBuffer, "v", heightKernel, velocityKernel, frictionKernel, collisionKernel);

            ComputeHelper.AssignTexture(computeShader, height_map, "Height_map", textureKernel);
            ComputeHelper.AssignTexture(computeShader, normal_map, "Normal_map", textureKernel);

            ComputeHelper.SetBuffer(computeShader, contactBuffer, "contacts", collisionKernel);


            // Set const
            computeShader.SetFloat("spacing", (float)spacing);
            computeShader.SetFloat("gravity", (float)gravity);
            computeShader.SetInt("numX", numX);
            computeShader.SetInt("numZ", numZ);

            foreach (var localKeywordName in computeShader.shaderKeywords)
            {
                Debug.Log("Local shader keyword " + localKeywordName + " is currently enabled");
            }
        }

        private void UpdateGPUSettings(REAL dt)
        {
            computeShader.SetFloat("dt", (float)dt);
            computeShader.SetFloat("gravity", (float)gravity);
            computeShader.SetFloat("mu", (float)mu);
        }
        private void AdvectionGPU()
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 0);
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 1);
        }

        private void UpdateVelocityGPU()
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: velocityKernel);
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: frictionKernel);
        }
        void UpdateVisMesh()
        {
            if (!updateMesh)
                return;
            //Update the height
            heightBuffer?.GetData(heights);
            for (int i = 0; i < this.numCells; i++)
            {
                meshVertices[i].y = (float)heights[i];
            }

            //Update the mesh
            mesh.SetVertices(meshVertices);
            //mesh.RecalculateNormals();
            //mesh.RecalculateTangents();
            mesh.RecalculateBounds();
        }

        void UpdateTextureGPU()
        {
            ComputeHelper.Dispatch(computeShader, height_map, textureKernel);
        }

    }
}