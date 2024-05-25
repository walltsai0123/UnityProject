using UnityEngine;
using Unity.Mathematics;
namespace XPBD
{
    public class SandSurface : Primitive
    {
        public XPBD.Plane Plane { get; private set; }

        [SerializeField] float sizeX;
        [SerializeField] float sizeZ;
        [SerializeField] float spacing;
        [SerializeField] float depth;
        [SerializeField] float dilute;
        [SerializeField] float ground_elavation;
        [SerializeField] float mu;
        [SerializeField] Material meshMaterial;

        int numX;
        int numZ;
        int numCells;

        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        //Mesh mesh;
        private Vector3[] meshVertices;

        float[] heights;
        float[] u;
        float[] v;

        float gravity = -9.81f;

        //Material material;
        RenderTexture height_map;
        RenderTexture normal_map;

        [SerializeField] ComputeShader computeShader;
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
            public float3 point;
            public float3 velocity;
            public float penetration;
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
        public override void Simulate(float dt)
        {
            Timer timer = new Timer();
            timer.Tic();
            UpdateGPUSettings(dt);
            AdvectionGPU(dt);
            UpdateVelocityGPU(dt);
            timer.Toc();

            //timer.Report("Time step: ", Timer.TimerOutputUnit.TIMER_OUTPUT_MILLISECONDS);
        }

        public override void UpdateVisual()
        {
            //UpdateVisMesh();
            UpdateTextureGPU();
        }

        public override void ApplyVelocity()
        {
            if(collisions.Count == 0)
                return;
            Contact [] contact = new Contact[collisions.Count];
            for(int i = 0; i < collisions.Count; i++)
            {
                var collision = collisions[i];
                Vector3 localPoint = transform.InverseTransformPoint(collision.q);
                Vector3 localVel = transform.InverseTransformDirection(collision.body.Vel[collision.index]);

                int X = Mathf.FloorToInt(localPoint.x / spacing) + Mathf.FloorToInt(numX / 2);
                int Z = Mathf.FloorToInt(localPoint.z / spacing) + Mathf.FloorToInt(numZ / 2);

                float pene = math.length(collision.q - collision.body.Pos[collision.index]);
                contact[i] = new Contact()
                {
                    index = X * numZ + Z,
                    point = localPoint,
                    velocity = localVel,
                    penetration = pene
                };
            }

            contactBuffer.SetData(contact);
            ComputeHelper.Dispatch(computeShader, collisions.Count, kernelIndex: collisionKernel);
        }

        private void Initialize()
        {
            numX = Mathf.FloorToInt(sizeX / spacing) + 1;
            numZ = Mathf.FloorToInt(sizeZ / spacing) + 1;

            numCells = numX * numZ;

            heights = new float[numCells];
            u = new float[numCells];
            v = new float[numCells];

            System.Array.Fill(heights, 0.01f);
            System.Array.Fill(u, 0f);
            System.Array.Fill(v, 0f);

            int dimX = Mathf.Min(30, numX / 4);
            int dimZ = Mathf.Min(30, numZ / 4);
            for (int i = numX / 2 - dimX; i < numX / 2 + dimX; i++)
            {
                for (int j = numZ / 2 - dimZ; j < numZ / 2 + dimZ; j++)
                {
                    int id = i * numZ + j;
                    heights[id] = depth;
                }

            }
            
            Plane = GetComponent<XPBD.Plane>();
            Plane.size = new Vector2(sizeX, sizeZ);
        }

        private void GenerateMesh()
        {
            //Generate the mesh's vertices and uvs
            Vector3[] positions = new Vector3[this.numCells];
            Vector2[] uvs = new Vector2[this.numCells];

            //Center of the mesh
            int cx = Mathf.FloorToInt(this.numX / 2f);
            int cz = Mathf.FloorToInt(this.numZ / 2f);

            for (int i = 0; i < this.numX; i++)
            {
                for (int j = 0; j < this.numZ; j++)
                {
                    float posX = (i - cx) * spacing;
                    float posY = 0f;
                    float posZ = (j - cz) * spacing;

                    positions[i * this.numZ + j] = new Vector3(posX, posY, posZ);

                    float u = i / (float)this.numX;
                    float v = j / (float)this.numZ;

                    uvs[i * this.numZ + j] = new Vector2(u, v);
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
            heightBuffer = new ComputeBuffer(numCells, sizeof(float));
            heightCacheBuffer = new ComputeBuffer(numCells, sizeof(float));
            uBuffer = new ComputeBuffer(numCells, sizeof(float));
            vBuffer = new ComputeBuffer(numCells, sizeof(float));
            contactBuffer = new ComputeBuffer(numCells, System.Runtime.InteropServices.Marshal.SizeOf(typeof(Contact)));

            heightBuffer.SetData(heights);
            uBuffer.SetData(u);
            vBuffer.SetData(v);

            ComputeHelper.SetBuffer(computeShader, heightBuffer, "heights", heightKernel, cacheKernel, velocityKernel, textureKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, heightCacheBuffer, "heights_cache", heightKernel, cacheKernel);

            ComputeHelper.SetBuffer(computeShader, uBuffer, "u", heightKernel, velocityKernel, frictionKernel, collisionKernel);
            ComputeHelper.SetBuffer(computeShader, vBuffer, "v", heightKernel, velocityKernel, frictionKernel, collisionKernel);

            ComputeHelper.AssignTexture(computeShader, height_map, "Height_map", textureKernel);
            ComputeHelper.AssignTexture(computeShader, normal_map, "Normal_map", textureKernel);

            ComputeHelper.SetBuffer(computeShader, contactBuffer, "contacts", collisionKernel);


            // Set const
            computeShader.SetFloat("spacing", spacing);
            computeShader.SetFloat("gravity", gravity);
            computeShader.SetInt("numX", numX);
            computeShader.SetInt("numZ", numZ);
        }

        private void UpdateGPUSettings(float dt)
        {
            computeShader.SetFloat("dt", dt);
            computeShader.SetFloat("gravity", gravity);
            computeShader.SetFloat("mu", mu);
        }
        private void AdvectionGPU(float dt)
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 0);
            //ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: 1);
        }

        private void UpdateVelocityGPU(float dt)
        {
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: velocityKernel);
            ComputeHelper.Dispatch(computeShader, numCells, kernelIndex: frictionKernel);
        }
        void UpdateVisMesh()
        {

            //Update the height
            heightBuffer?.GetData(heights);
            for (int i = 0; i < this.numCells; i++)
            {
                meshVertices[i].y = heights[i];
            }

            //Update the mesh
            mesh.SetVertices(meshVertices);
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
        }

        void UpdateTextureGPU()
        {
            ComputeHelper.Dispatch(computeShader, height_map, textureKernel);
        }

    }
}