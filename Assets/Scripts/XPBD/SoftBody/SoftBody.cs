using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using System.Collections.Generic;
using System;
using UnityEngine.Assertions;
using System.Linq;


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
    public struct SoftBodyParticle
    {
        public REAL3 pos;
        public REAL3 prevPos;
        public REAL3 vel;
        public REAL invMass;
        public REAL Mass
        {
            get
            {
                return (invMass < Util.EPSILON) ? 0 : 1 / invMass;
            }
        }

        public REAL3 f_ext;

        public REAL radius;
        public REAL surfaceArea
        {
            get
            {
                return 4 * math.PI * radius * radius;
            }
        }

        public REAL volume
        {
            get
            {
                return (REAL)(4.0 / 3.0) * math.PI * radius * radius * radius;
            }
        }
        public SoftBodyParticle(REAL3 position, REAL inverseMass, REAL volume)
        {
            pos = prevPos = position;
            vel = 0;
            invMass = inverseMass;
            f_ext = 0;
            radius = math.pow((REAL)(3.0 / 4.0) * volume / math.PI, (REAL)(1.0 / 3.0));
        }
    };
    public struct ElementConstraint
    {
        public bool active;
        public int4 tet;
        public REAL3x3 invDm;
        public REAL restVolume;
        public REAL mu, lambda;

        public ElementConstraint(int4 tetId, REAL3x3 inverseDm, REAL volume, REAL Mu, REAL Lambda)
        {
            active = true;
            tet = tetId;
            invDm = inverseDm;
            restVolume = volume;
            mu = Mu;
            lambda = Lambda;
        }

        public bool AreAdjacent(ElementConstraint other)
        {
            int sharedVertices = 0;
            // check vertex equality
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    if (tet[i] == other.tet[j])
                        sharedVertices++;

                    if (sharedVertices >= 1)
                        return true;
                }
            }
            return false;
        }

        public float Distance(ElementConstraint other)
        {
            int intersect = 0;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    if (tet[i] == other.tet[j])
                        intersect++;
                }
            }
            float union = 8 - intersect;

            // distance = 1 - |Cj ¡ä Ck| / |Cj ¡å Ck|
            return 1f - intersect / union;
        }

        public override readonly string ToString()
        {
            string s = "";
            s += active + "\n";
            s += tet + "\n";
            s += restVolume + "\n";
            s += $"{mu} {lambda}\n";
            s += invDm + "\n";
            s += base.ToString();
            return s;
        }
    };

    public class SoftBody : Body
    {
        [SerializeField] TetrahedronMesh tetrahedronMesh;
        private MeshRenderer meshRenderer;
        private MeshFilter meshFilter;
        public Mesh visualMesh;
        public Mesh tetMesh;
        public Material visualMaterial { get; private set; }
        public Material collisionMaterial { get; private set; }
        [SerializeField] Material wireframeMaterial;

        public REAL mu = 10f, lambda = 1000f;
        public bool showTet = false;
        public bool swapZ = false;

        // Simulation Data NativeArray
        public NativeArray<REAL3> Pos;
        public NativeArray<REAL3> prevPos;
        public NativeArray<REAL3> Vel;
        public NativeArray<int4> tets;
        public NativeArray<REAL> invMass;
        public NativeArray<REAL> restVolumes;
        public NativeArray<REAL3x3> invDm;
        private NativeArray<int2> edges;
        private NativeArray<REAL> restEdgeLengths;

        public List<SoftBodyParticle> particles;
        public List<ElementConstraint> elementConstraints;
        public List<int3> surfaceFaces;

        private NativeArray<int3> volIdOrder;
        public REAL3 fext { get; set; } = REAL3.zero;

        // Skinning Info
        private NativeArray<Vector3> visPos;
        public NativeArray<REAL4> skinningInfo;

        public int VerticesNum { get; private set; }
        public int TetsNum { get; private set; }

        public int FacesNum { get; private set; }

        private int EdgesNum;

        [SerializeField]
        private REAL3 v0 = REAL3.zero;
        [SerializeField]
        private REAL3 translate = REAL3.zero;

        // Body forward direction
        public REAL3 X_COM;
        public REAL3 V_COM;
        public REAL3 forwardDir = new (0,0,1);

        private JobHandle jobHandle;

        public List<MyCollision> collisions = new();

        // Grabber info
        public int grabId { get; private set; }
        private REAL grabInvMass;

        int[] elementConstraitIds;
        int[] passSize;

        REAL3 startPos;

        [SerializeField,DebugOnly] float f_N = 0;
        [SerializeField, DebugOnly] float Energy = 0;
        [SerializeField, DebugOnly] float Energy_H = 0;
        [SerializeField, DebugOnly] float Energy_D = 0;

        public void SetForce(REAL3 F)
        {
            for(int i = 0; i < VerticesNum; i++)
            {
                var p = particles[i];
                p.f_ext = F;
                particles[i] = p;
            }
        }
        public void ClearForce()
        {
            SetForce(0);
        }
        #region Body

        public override void EndFrame()
        {
            Energy = (float)GetEnergy();
            X_COM = 0;
            V_COM = 0;
            foreach(var p in particles)
            {
                X_COM += p.pos * p.Mass;
                V_COM += p.vel * p.Mass;
            }
            X_COM *= this.InvMass;
            V_COM *= this.InvMass;

            if (collisions.Count > 0)
            {
                float3 fN = 0.0f;
                foreach (var C in collisions)
                    fN += (float3)C.F * new float3(0, 1, 0);

                f_N = math.length(fN / Simulation.get.substeps);
            }

            REAL3 lastDir = forwardDir;
            forwardDir = math.normalizesafe(V_COM, lastDir);

            if (showTet || visualMesh == null)
            {
                meshFilter.mesh = tetMesh;
                meshRenderer.material = wireframeMaterial;

                Vector3[] V = new Vector3[VerticesNum];
                for (int i = 0; i < VerticesNum; i++)
                    V[i] = (float3)Pos[i];
                tetMesh.SetVertices(V);
                tetMesh.RecalculateBounds();
                tetMesh.RecalculateNormals();
                tetMesh.RecalculateTangents();
            }
            else
            {
                meshFilter.mesh = visualMesh;
                meshRenderer.material = visualMaterial;

                UpdateVisMeshJob updateVisMeshJob = new UpdateVisMeshJob
                {
                    visPos = visPos,
                    pos = Pos,
                    tets = tets,
                    skinningInfo = skinningInfo
                };
                jobHandle = updateVisMeshJob.Schedule(visPos.Length, 1, jobHandle);
                jobHandle.Complete();

                visualMesh.SetVertices(visPos);
                visualMesh.RecalculateBounds();
                visualMesh.RecalculateNormals();
                visualMesh.RecalculateTangents();
            }
        }
        #endregion

        #region IGrabbable
        public override void StartGrab(REAL3 grabPos)
        {
            //Find the closest vertex to the pos on a triangle in the mesh
            REAL minD2 = REAL.MaxValue;
            REAL3 gPos = grabPos;
            grabId = -1;

            for (int i = 0; i < VerticesNum; i++)
            {

                REAL d2 = math.lengthsq(gPos - Pos[i]);

                if (d2 < minD2)
                {
                    minD2 = d2;
                    grabId = i;
                }
            }

            //We have found a vertex
            if (grabId >= 0)
            {
                //Save the current innverted mass
                grabInvMass = invMass[grabId];

                //Set the inverted mass to 0 to mark it as fixed
                invMass[grabId] = 0;

                //Set the position of the vertex to the position where the ray hit the triangle
                Pos[grabId] = grabPos;
            }
        }

        public override void MoveGrabbed(REAL3 grabPos)
        {
            if (grabId >= 0)
            {
                Pos[grabId] = grabPos;
            }
        }

        public override void EndGrab(REAL3 grabPos, REAL3 vel)
        {
            if (grabId >= 0)
            {
                //Set the mass to whatever mass it was before we grabbed it
                invMass[grabId] = grabInvMass;

                this.Vel[grabId] = 0;
            }

            grabId = -1;
        }

        public override void IsRayHittingBody(Ray ray, out CustomHit hit)
        {
            REAL3[] vertices = Pos.ToArray();
            int[] triangles = tetrahedronMesh.faces;

            Intersection.IsRayHittingMesh(ray, vertices, triangles, out hit);
        }

        public override REAL3 GetGrabbedPos()
        {
            return (float3)Pos[grabId];
        }
        #endregion

        #region Monobehaviour
        private void Awake()
        {
            Assert.IsNotNull(tetrahedronMesh);

            InitializeMesh();

            // Move to starting position
            startPos = (float3)transform.position;
            transform.position = Vector3.zero;
        }
        private void OnEnable()
        {
            InitializePhysics();
            EndFrame();
            Simulation.get.AddBody(this);
        }

        private void OnDisable()
        {
            if (Pos.IsCreated) Pos.Dispose();
            if (prevPos.IsCreated) prevPos.Dispose();
            if (Vel.IsCreated) Vel.Dispose();
            if (invMass.IsCreated) invMass.Dispose();
            if (tets.IsCreated) tets.Dispose();
            if (restVolumes.IsCreated) restVolumes.Dispose();
            if (invDm.IsCreated) invDm.Dispose();
            if (visPos.IsCreated) visPos.Dispose();
            if (skinningInfo.IsCreated) skinningInfo.Dispose();

            if (edges.IsCreated) edges.Dispose();
            if (restEdgeLengths.IsCreated) restEdgeLengths.Dispose();
            if (volIdOrder.IsCreated) volIdOrder.Dispose();
        }
        #endregion

        private void InitializeMesh()
        {
            meshFilter = GetComponent<MeshFilter>();
            meshRenderer = GetComponent<MeshRenderer>();

            if(meshFilter.sharedMesh != null)
            {
                visualMesh = meshFilter.mesh;
            }
                

            // Create tetmesh from tetrahedron data
            tetMesh = new Mesh();
            tetMesh.name = gameObject.name + "_tetmesh";
            tetMesh.SetVertices(tetrahedronMesh.vertices);
            tetMesh.SetIndices(tetrahedronMesh.faces, MeshTopology.Triangles, 0);
            tetMesh.RecalculateNormals();

            Color[] colors = new Color[tetMesh.vertices.Length];

            for (int i = 0; i < colors.Length; i++)
            {
                colors[i] = new Color(1, 1, 1, 0.75f);
            }
            for (int i = 0; i < tetMesh.triangles.Length; i++)
            {
                colors[tetMesh.triangles[i]] = new Color(1, 0, 0, .25f);
            }
            tetMesh.SetColors(colors);

            // Set materials
            visualMaterial = meshRenderer.material;
            collisionMaterial = new(Simulation.get.textureFrictionShader);

        }
        private void InitializePhysics()
        {
            bodyType = BodyType.Soft;

            VerticesNum = tetrahedronMesh.vertices.Length;
            TetsNum = tetrahedronMesh.tets.Length / 4;
            FacesNum = tetrahedronMesh.faces.Length / 3;
            EdgesNum = tetrahedronMesh.edges.Length / 2;


            Pos = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            prevPos = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            Vel = new NativeArray<REAL3>(VerticesNum, Allocator.Persistent);
            invMass = new NativeArray<REAL>(VerticesNum, Allocator.Persistent);
            REAL[] volumes = new REAL[VerticesNum];

            tets = new NativeArray<int4>(TetsNum, Allocator.Persistent);
            restVolumes = new NativeArray<REAL>(TetsNum, Allocator.Persistent);
            invDm = new NativeArray<REAL3x3>(TetsNum, Allocator.Persistent);

            edges = new NativeArray<int2>(EdgesNum, Allocator.Persistent);
            restEdgeLengths = new NativeArray<REAL>(EdgesNum, Allocator.Persistent);

            volIdOrder = new NativeArray<int3>(4, Allocator.Persistent);
            volIdOrder[0] = new(1, 3, 2);
            volIdOrder[1] = new(0, 2, 3);
            volIdOrder[2] = new(0, 3, 1);
            volIdOrder[3] = new(0, 1, 2);

            // Initialize NativeArrays
            for (int i = 0; i < VerticesNum; ++i)
            {
                float3 vert = (float3)tetrahedronMesh.vertices[i];
                if(swapZ)
                {
                    math.rotate(quaternion.AxisAngle(new float3(0, 1, 0), math.PI), vert); 
                }
                Pos[i] = vert;
                Pos[i] += startPos;
                prevPos[i] = Pos[i];
                Vel[i] = v0;
                invMass[i] = 0f;
            }

            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tetrahedronMesh.tets[4 * i + 0];
                int id1 = tetrahedronMesh.tets[4 * i + 1];
                int id2 = tetrahedronMesh.tets[4 * i + 2];
                int id3 = tetrahedronMesh.tets[4 * i + 3];

                tets[i] = new int4(id0, id1, id2, id3);
            }
            for (int i = 0; i < TetsNum; ++i)
            {
                restVolumes[i] = 0f;
                invDm[i] = REAL3x3.identity;
            }
            for(int i = 0; i < EdgesNum; ++i)
            {
                int id0 = tetrahedronMesh.edges[2 * i + 0];
                int id1 = tetrahedronMesh.edges[2 * i + 1];

                edges[i] = new int2(id0, id1);
                restEdgeLengths[i] = math.length(Pos[id0] - Pos[id1]);
            }

            // Rest volume
            REAL totalVolume = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;


                REAL3x3 RestPose = new (Pos[id1] - Pos[id0], Pos[id2] - Pos[id0], Pos[id3] - Pos[id0]);
                invDm[i] = math.inverse(RestPose);
                REAL V = math.determinant(RestPose) / 6f;

                REAL partialV = V / 4f;
                invMass[id0] += partialV;
                invMass[id1] += partialV;
                invMass[id2] += partialV;
                invMass[id3] += partialV;

                restVolumes[i] = V;
                totalVolume += V;
            }

            
            // Inverse mass (1/w)
            REAL density = mass / totalVolume;
            for (int i = 0; i < VerticesNum; ++i)
            {
                volumes[i] = invMass[i];
                invMass[i] *= density;
                if (invMass[i] != 0f)
                    invMass[i] = 1f / invMass[i];
            }


            particles = new List<SoftBodyParticle>(VerticesNum);
            elementConstraints = new List<ElementConstraint>(TetsNum);
            surfaceFaces = new List<int3>(tetrahedronMesh.faces.Length / 3);

            for (int i = 0; i < VerticesNum; ++i)
                particles.Add(new SoftBodyParticle(Pos[i], invMass[i], volumes[i]));
            for (int i = 0; i < TetsNum; ++i)
                elementConstraints.Add(new ElementConstraint(tets[i], invDm[i], restVolumes[i], mu, lambda));
            for (int i = 0; i < tetrahedronMesh.faces.Length; i += 3)
                surfaceFaces.Add(new int3(tetrahedronMesh.faces[i], tetrahedronMesh.faces[i + 1], tetrahedronMesh.faces[i + 2]));

            // Visual embedded mesh
            ComputeSkinningInfo(startPos);

            // Do graph coloring to set up element constraint passes
            int[] colors = tetrahedronMesh.GraphColoring();
            int[] colorsize = new int[colors.Length];

            Assert.IsTrue(colors.Length == TetsNum);

            int[] indices = new int[tets.Length];
            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = i;
            }

            // Sort indices in order of color
            Array.Sort(indices, (i1, i2) => colors[i1].CompareTo(colors[i2]));

            List<int> passSizeList = new List<int>();

            int currentColor = -1;
            for(int i = 0; i < indices.Length; i++)
            {
                // Get color of tet
                int color = colors[indices[i]];

                // check if the color is new
                if(currentColor != color)
                {
                    // Add new color
                    currentColor = color;
                    passSizeList.Add(0);
                }

                // Increase the color count
                passSizeList[passSizeList.Count - 1]++;
            }

            elementConstraitIds = indices.ToArray();
            passSize = passSizeList.ToArray();
        }

        private void ComputeSkinningInfo(REAL3 startPos)
        {
            if(visualMesh == null)
                return;
            int numVisVerts = visualMesh.vertexCount;
            visPos = new NativeArray<Vector3>(visualMesh.vertexCount, Allocator.Persistent);

            REAL3[] vertices = new REAL3[numVisVerts];
            for (int i = 0; i < numVisVerts; i++)
            {
                vertices[i] = (float3)visualMesh.vertices[i] + startPos;
            }

            Hash hash = new Hash(0.15f, numVisVerts);
            hash.Create(vertices);

            skinningInfo = new NativeArray<REAL4>(numVisVerts, Allocator.Persistent);
            for (int i = 0; i < skinningInfo.Length; ++i)
                skinningInfo[i] = new REAL4(-1, -1, -1, -1);

            REAL[] minDist = new REAL[numVisVerts];
            Array.Fill(minDist, REAL.MaxValue);
            REAL border = 0.05f;

            // Each tet searches for containing vertices
            REAL3 tetCenter;
            REAL3x3 mat = REAL3x3.zero;
            REAL4 bary = REAL4.zero;
            //REAL averageEdge = 0f;
            for (int i = 0; i < TetsNum; ++i)
            {
                // Compute bounding sphere for tet
                tetCenter = REAL3.zero;
                for (int j = 0; j < 4; ++j)
                    tetCenter += 0.25f * (Pos[tets[i][j]]);

                REAL rMax = 0f;

                for (int j = 0; j < 4; ++j)
                {
                    REAL r2 = math.length(tetCenter - Pos[tets[i][j]]);
                    rMax = math.max(rMax, r2);
                }

                rMax += border;

                hash.Query(tetCenter, rMax);
                if (hash.querySize == 0)
                    continue;

                int id0 = tets[i].x;
                int id1 = tets[i].y;
                int id2 = tets[i].z;
                int id3 = tets[i].w;

                mat.c0 = Pos[id0] - Pos[id3];
                mat.c1 = Pos[id1] - Pos[id3];
                mat.c2 = Pos[id2] - Pos[id3];

                mat = math.inverse(mat);

                for (int j = 0; j < hash.querySize; ++j)
                {
                    int id = hash.queryIds[j];

                    if (minDist[id] <= 0f)
                        continue;

                    REAL3 visVert = (float3)vertices[id];
                    if (math.lengthsq(visVert - tetCenter) > rMax * rMax)
                        continue;

                    bary.xyz = visVert - Pos[id3];
                    bary.xyz = math.mul(mat, bary.xyz);
                    bary.w = 1f - bary.x - bary.y - bary.z;

                    REAL dist = 0f;

                    for (int k = 0; k < 4; ++k)
                        dist = math.max(dist, -bary[k]);

                    if (dist < minDist[id])
                    {
                        minDist[id] = dist;
                        skinningInfo[id] = new REAL4(i, bary[0], bary[1], bary[2]);
                    }
                }
            }
        }

        private REAL GetEnergy()
        {
            REAL energy_D = 0f;
            REAL energy_V = 0f;
            REAL energy = 0f;
            REAL gamma = mu / lambda;

            REAL3x3 PK1 = REAL3x3.zero;
            for (int i = 0; i < TetsNum; i++)
            {
                REAL3x3 F = GetDeformationGradient(i);
                REAL J = math.determinant(F);
                REAL energyD = 0.5f * mu * (math.lengthsq(F.c0) + math.lengthsq(F.c1) + math.lengthsq(F.c2) - 3f);
                REAL energyV = 0.5f * lambda * math.pow(J - 1f, 2f);
                energy_D += energyD * restVolumes[i];
                energy_V += energyV * restVolumes[i];
                energy += (energyD + energyV) * restVolumes[i];

                REAL3x3 PJPF = new REAL3x3(math.cross(F.c1, F.c2), math.cross(F.c2, F.c0), math.cross(F.c0, F.c1));
                PK1 += mu * F + PJPF * (lambda * (J - 1f) - mu);
            }

            return energy;

            REAL3x3 GetDeformationGradient(int tetIndex)
            {
                int id0 = tets[tetIndex].x;
                int id1 = tets[tetIndex].y;
                int id2 = tets[tetIndex].z;
                int id3 = tets[tetIndex].w;

                REAL3x3 Ds = REAL3x3.zero;
                Ds.c0 = Pos[id1] - Pos[id0];
                Ds.c1 = Pos[id2] - Pos[id0];
                Ds.c2 = Pos[id3] - Pos[id0];

                return math.mul(Ds, invDm[tetIndex]);
            }
        }

        private void Translate(REAL x, REAL y, REAL z)
        {
            for(int i = 0; i < VerticesNum; i++)
            {
                Pos[i] += new REAL3(x, y, z);
                prevPos[i] = Pos[i];
            }
        }



        #region IJob

        [BurstCompile]
        private struct UpdateVisMeshJob : IJobParallelFor
        {
            public NativeArray<Vector3> visPos;
            [ReadOnly]
            public NativeArray<REAL3> pos;
            [ReadOnly]
            public NativeArray<int4> tets;
            [ReadOnly]
            public NativeArray<REAL4> skinningInfo;

            public void Execute(int index)
            {
                int tetNr = (int)skinningInfo[index][0];
                if (tetNr < 0)
                    return;

                REAL b0 = skinningInfo[index][1];
                REAL b1 = skinningInfo[index][2];
                REAL b2 = skinningInfo[index][3];
                REAL b3 = 1f - b0 - b1 - b2;

                int id0 = tets[tetNr].x;
                int id1 = tets[tetNr].y;
                int id2 = tets[tetNr].z;
                int id3 = tets[tetNr].w;

                visPos[index] = Vector3.zero;
                visPos[index] = (float3)visPos[index] + (float3)(pos[id0] * b0);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id1] * b1);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id2] * b2);
                visPos[index] = (float3)visPos[index] + (float3)(pos[id3] * b3);
            }
        }
        #endregion
    }

}

