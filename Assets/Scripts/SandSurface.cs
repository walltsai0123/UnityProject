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
    public class SandSurface : HeightMapMesh
    {
        //public XPBD.Plane Plane { get; private set; }

        //[SerializeField] REAL sizeX;
        //[SerializeField] REAL sizeZ;
        //[SerializeField] REAL spacing;
        //[SerializeField] REAL depth;
        //[SerializeField] REAL dilute;
        //[SerializeField] REAL ground_elavation;
        //[SerializeField] REAL mu;
        //[SerializeField] Material meshMaterial;
        //[SerializeField] ComputeShader computeShader;
        //[SerializeField] private bool updateMesh = false;

        //int numX;
        //int numZ;
        int numCells;

        //private MeshRenderer meshRenderer;
        //private MeshFilter meshFilter;
        //Mesh mesh;
        private Vector3[] meshVertices;

        const int meshX = 10;
        const int meshZ = 10;

        REAL[] heights;
        REAL[] u;
        REAL[] v;
        REAL totalHeight = 0f;
        REAL maxHeight;
        readonly REAL gravity = -9.81f;

        //Material material;
        //RenderTexture height_map;
        //RenderTexture normal_map;
        
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

        protected void Start()
        {
            //base.Start();

            //Initialize();
            //GenerateMesh();
            //GenerateTexture();

            //InitComputeBuffers();
        }

        private void OnDestroy()
        {
            ComputeHelper.Release(heightBuffer, heightCacheBuffer, uBuffer, vBuffer, contactBuffer);
        }
        //public override void Simulate(REAL dt)
        //{
        //    UpdateGPUSettings(dt);
        //    AdvectionGPU();
        //    UpdateVelocityGPU();
        //}

        //public override void UpdateVisual()
        //{
        //    //UpdateVisMesh();
        //    UpdateTextureGPU();
        //}

        public  void ApplyVelocity(REAL dt)
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

        public override void Initialize(int width, float Size)
        {
            base.Initialize(width, Size);

            height_map.enableRandomWrite = true;
            normal_map.enableRandomWrite = true;

            //numX = (int)math.floor(sizeX / spacing) + 1;
            //numZ = (int)math.floor(sizeZ / spacing) + 1;

            //numCells = numX * numZ;

            //heights = new REAL[numCells];
            //u = new REAL[numCells];
            //v = new REAL[numCells];
            //totalHeight = 0;

            //System.Array.Fill(heights, 0.01f);
            //System.Array.Fill(u, 0f);
            //System.Array.Fill(v, 0f);

            //int dimX = Mathf.Min(30, numX / 4);
            //int dimZ = Mathf.Min(30, numZ / 4);
            //for (int i = 0; i < numX; i++)
            //{
            //    for (int j = 0; j < numZ; j++)
            //    {
            //        int id = i * numZ + j;
            //        heights[id] = UnityEngine.Random.Range(0.01f, (float)depth);
            //        heights[id] = 0.01f;
            //        if (j < numZ / 2)
            //            heights[id] = depth;
            //    }
            //}

            //maxHeight = 0f;
            //for (int i = 0; i < numCells; i++)
            //{
            //    totalHeight += heights[i];
            //    maxHeight = math.max(maxHeight, heights[i]);
            //}

            //maxHeight *= 2;


            GenerateTexture();
        }

        private void GenerateMesh()
        {
            //meshFilter.mesh = mesh = new Mesh();
            //mesh.name = "Procedural Grid";

            //Vector3[] vertices = new Vector3[(meshX + 1) * (meshZ + 1)];
            //Vector2[] uv = new Vector2[vertices.Length];

            //float deltaX = (float)sizeX / meshX;
            //float deltaY = (float)sizeZ / meshZ;
            //for (int i = 0, y = 0; y <= meshZ; y++)
            //{
            //    for (int x = 0; x <= meshZ; x++, i++)
            //    {
            //        vertices[i] = new Vector3(x * deltaX, 0, y * deltaY);

            //        uv[i] = new Vector2((float)x / meshX, (float)y / meshZ);
            //    }
            //}
            //mesh.vertices = vertices;

            //int[] triangles = new int[meshX * meshZ * 6];
            //for (int ti = 0, vi = 0, y = 0; y < meshZ; y++, vi++)
            //{
            //    for (int x = 0; x < meshX; x++, ti += 6, vi++)
            //    {
            //        triangles[ti] = vi;
            //        triangles[ti + 3] = triangles[ti + 2] = vi + 1;
            //        triangles[ti + 4] = triangles[ti + 1] = vi + meshX + 1;
            //        triangles[ti + 5] = vi + meshX + 2;
            //    }
            //}
            //mesh.triangles = triangles;
            //mesh.uv = uv;

            //Bounds bounds = meshRenderer.bounds;
            //Vector3 newSize = bounds.size;
            //newSize.y = 100;
            //meshRenderer.bounds = new Bounds(bounds.center, newSize);
        }

        private void GenerateTexture()
        {
            //material = meshRenderer.material;
            //height_map = new RenderTexture(numX, numZ, 32);
            //normal_map = new RenderTexture(numX - 1, numZ - 1, 32);

            //height_map.enableRandomWrite = true;
            //normal_map.enableRandomWrite = true;

            //height_map.Create();
            //normal_map.Create();

            //SetMaterialProperties();

            //material.SetTexture("_ParallaxMap", height_map);
            //material.SetTexture("_TerrainNormalMap", normal_map);
            //material.SetFloat("_Parallax", (float)maxHeight);
        }

        private void InitComputeBuffers()
        {
            //heightBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            //heightCacheBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            //uBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            //vBuffer = new ComputeBuffer(numCells, sizeof(REAL));
            //contactBuffer = new ComputeBuffer(numCells, System.Runtime.InteropServices.Marshal.SizeOf(typeof(Contact)));
            //
            //heightBuffer.SetData(heights);
            //uBuffer.SetData(u);
            //vBuffer.SetData(v);
            //
            //ComputeHelper.SetBuffer(computeShader, heightBuffer, "heights", heightKernel, cacheKernel, velocityKernel, textureKernel, collisionKernel);
            //ComputeHelper.SetBuffer(computeShader, heightCacheBuffer, "heights_cache", heightKernel, cacheKernel);
            //
            //ComputeHelper.SetBuffer(computeShader, uBuffer, "u", heightKernel, velocityKernel, frictionKernel, collisionKernel);
            //ComputeHelper.SetBuffer(computeShader, vBuffer, "v", heightKernel, velocityKernel, frictionKernel, collisionKernel);
            //
            //ComputeHelper.AssignTexture(computeShader, height_map, "Height_map", textureKernel);
            //ComputeHelper.AssignTexture(computeShader, normal_map, "Normal_map", textureKernel);
            //
            //ComputeHelper.SetBuffer(computeShader, contactBuffer, "contacts", collisionKernel);
            //
            //
            //// Set const
            //computeShader.SetFloat("spacing", (float)spacing);
            //computeShader.SetFloat("gravity", (float)gravity);
            //computeShader.SetInt("numX", numX);
            //computeShader.SetInt("numZ", numZ);
            //computeShader.SetFloat("maxHeight", (float)maxHeight);
            //
            //foreach (var localKeywordName in computeShader.shaderKeywords)
            //{
            //    Debug.Log("Local shader keyword " + localKeywordName + " is currently enabled");
            //}
        }

        //private void UpdateGPUSettings(REAL dt)
        //{
        //    computeShader.SetFloat("dt", (float)dt);
        //    computeShader.SetFloat("gravity", (float)gravity);
        //    computeShader.SetFloat("mu", (float)mu);
        //}
        //private void AdvectionGPU()
        //{
        //    ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 0);
        //    ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 1);
        //}

        //private void UpdateVelocityGPU()
        //{
        //    ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: velocityKernel);
        //    ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: frictionKernel);
        //}

        //void UpdateTextureGPU()
        //{
        //    ComputeHelper.Dispatch(computeShader, height_map, textureKernel);
        //}

    }
}