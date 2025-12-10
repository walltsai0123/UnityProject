using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;
using UnityEngine.TerrainTools;
using System.Linq;
using System.Collections;
using UnityEngine.Rendering;
using UnityEditor;
using System.Runtime.ConstrainedExecution;
using System.IO;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;
//using UnityEditor;

namespace XPBD
{
    public class MyTerrain : MonoBehaviour
    {
        public Terrain Terrain;

        private TerrainData terrainData;
        private TerrainCollider m_Collider;

        private int heightmapWidth;
        private int heightmapHeight;
        private float terrainWidth;
        private float terrainLength;
        private float terrainHeight;
        public float CellWidth => terrainWidth / (heightmapWidth - 1);
        public float CellArea => CellWidth * CellWidth;

        [SerializeField] private RenderTexture originalHeightmap;
        [SerializeField] float maxContactTime = 10f;
        [SerializeField] float minContactTime = 0.05f;
        [SerializeField] float contactK = 0.1f;
        //[SerializeField] float L_0 = 0.3f;
        [SerializeField] int neighbourSearch = 2;
        [SerializeField] int bumpOffset = 5;

        [Header("Footprint Settings")]
        [SerializeField] bool printConvexHull = false;
        [SerializeField] bool convexHull = true;
        [SerializeField] bool useSinkage = true;
        [SerializeField] bool useSlipSinkage = true;
        [SerializeField] bool printFootPrint = true;
        [SerializeField] bool printLaterelBumps = true;
        [SerializeField] bool directionBump = true;
        [SerializeField] Material footprintMat;
        [DebugOnly, SerializeField, Min(0)] private float m_BrushOpacity = 0.1f;
        [DebugOnly, SerializeField] private float m_BrushSize = 0.01f;
        float2 m_brushUV;
        [DebugOnly] public Texture2D brushTexture;

        Dictionary<int2, float> gridDisplacements = new();

        public GroundMaterial GroundMaterial => groundMaterial;
        [SerializeField] GroundMaterial groundMaterial;

        [Header("Gaussian Filter")]
        [SerializeField]
        bool filter = true;
        [SerializeField] Material gaussianMaterial;

        [Header("Verbose")]
        [SerializeField] float bumpTime = 0;
        [SerializeField] float bumpTimeAvg = 0;
        int bumpFrames = 0;

        public List<MyCollision> collisions = new List<MyCollision>();

        Timer coroutineTimer = new();

        int times = 0;
        #region Terrain Paint
        public void TimerReport(int frame)
        {
            coroutineTimer.Toc();
            Debug.Log(name + " terrainPaint Time: " + coroutineTimer.DurationInSeconds() * 1000 / frame);
        }
        private IEnumerator PaintFootPrint()
        {
            coroutineTimer.Resume();
            if (gridDisplacements.Count <= 0)
            {
                coroutineTimer.Pause();
                yield return null;
            }
            GridDisplacements2BrushTexture();
            // Get the current BrushTransform under the mouse position relative to the Terrain
            BrushTransform brushXform = TerrainPaintUtility.CalculateBrushTransform(Terrain, m_brushUV, m_BrushSize, 0);

            // Get the PaintContext for the current BrushTransform. This has a sourceRenderTexture from which to read existing Terrain brushTexture data
            // and a destinationRenderTexture into which to write new Terrain brushTexture data
            PaintContext paintContext = TerrainPaintUtility.BeginPaintHeightmap(Terrain, brushXform.GetBrushXYBounds());

            // Get the built-in painting Material reference
            Material mat = footprintMat;
            // Bind the current brush brushTexture
            mat.SetTexture("_BrushTex", brushTexture);

            var opacity = m_BrushOpacity;
            mat.SetVector("_BrushParams", new Vector4(opacity, 0.0f, 0.0f, 0.0f));
            mat.SetFloat("terrainHeight", terrainHeight);
            // Setup the material for reading from/writing into the PaintContext brushTexture data. This is a necessary step to setup the correct shader properties for appropriately transforming UVs and sampling textures within the shader
            TerrainPaintUtility.SetupTerrainToolMaterialProperties(paintContext, brushXform, mat);
            // Render into the PaintContext's destinationRenderTexture using the built-in painting Material - the id for the Raise/Lower pass is 0.
            Graphics.Blit(paintContext.sourceRenderTexture, paintContext.destinationRenderTexture
                    , mat, (int)TerrainBuiltinPaintMaterialPasses.RaiseLowerHeight);
            // Commit the modified PaintContext with a provided string for tracking Undo operations. This function handles Undo and resource cleanup for you
            TerrainPaintUtility.EndPaintHeightmap(paintContext, "Terrain Paint - Raise or Lower Height");

            if (filter)
            {
                BrushTransform filterbrush = brushXform;
                PaintContext filterPC = TerrainPaintUtility.BeginPaintHeightmap(Terrain, filterbrush.GetBrushXYBounds());
                TerrainPaintUtility.SetupTerrainToolMaterialProperties(filterPC, filterbrush, gaussianMaterial);
                Graphics.Blit(filterPC.sourceRenderTexture, filterPC.destinationRenderTexture, gaussianMaterial);
                TerrainPaintUtility.EndPaintHeightmap(filterPC, "");
            }

            PaintContext.ApplyDelayedActions();

            coroutineTimer.Pause();
            yield return null;

            void GridDisplacements2BrushTexture()
            {

                // Create brush texture, Red: Lower, Green: Raise
                brushTexture = brushTexture != null ? brushTexture : new Texture2D(1, 1, TextureFormat.RG16, -1, false);
                var gridHeights = gridDisplacements;
                if (gridHeights.Count == 0)
                {
                    m_brushUV = 0f;
                    m_BrushSize = 0f;
                    m_BrushOpacity = 0f;
                    return;
                }
                (int2 localMin, int2 localMax) = gridHeights.Aggregate(
                    (localMin: gridHeights.First().Key, localMax: gridHeights.First().Key),
                    (acc, next) => (math.min(acc.localMin, next.Key), math.max(acc.localMax, next.Key))
                );
                int2 centerGrid = (localMin + localMax) / 2;
                int brushResolution = math.cmax(localMax - localMin);
                brushResolution += (brushResolution % 2) == 1 ? 4 : 3;
                brushResolution = math.max(65, brushResolution);

                brushTexture.Reinitialize(brushResolution, brushResolution);

                Color32[] colors = new Color32[brushResolution * brushResolution];
                float maxDisplacement = gridHeights.Values.Max();
                maxDisplacement = gridHeights.Values.Aggregate(
                    (max, next) =>
                    (math.abs(max) < math.abs(next) ? math.abs(next) : math.abs(max))
                );
                foreach (var height in gridHeights)
                {
                    int2 grid0 = centerGrid - brushResolution / 2;
                    int2 uv = height.Key - grid0;

                    Color32 maxColor = (height.Value < 0) ? Color.red : Color.green;
                    Color32 black = Color.black;
                    colors[uv.x + uv.y * brushResolution] = Color32.Lerp(black, maxColor, math.abs(height.Value) / maxDisplacement);
                }
                brushTexture.SetPixels32(colors);
                brushTexture.Apply();

                m_brushUV = ((float2)(localMin + localMax) / 2f / new float2((heightmapWidth - 1), (heightmapHeight - 1)));
                m_BrushSize = Grid2WorldGrid(brushResolution).x;
                m_BrushOpacity = maxDisplacement;

                gridDisplacements.Clear();
                return;
            }
        }
        public void PaintFootPrints(SoftBodySystem sbs, float dt)
        {
            if (!printFootPrint)
                return;

            float plotTotalForce = 0;
            float plotTotalArea = 0;
            float plotTotalSinkage = 0;
            float plotTotalSlipSinkage = 0;
            float plotTotalShearDisplace = 0;

            HashSet<int2> gridPoints = new HashSet<int2>();
            HashSet<int2> convexGridPoints = new HashSet<int2>();

            int bodyCount = sbs.softBodies.Count;
            if (bodyCount == 0)
            {
                Debug.Log("No soft bodies");
                return;
            }
            // grid cell area
            float lenghtCellX = CellWidth;
            float lenghtCellZ = CellWidth;
            float areaCell = CellArea;

            for (int i = 0; i < bodyCount; i++)
            {
                SoftBody softBody = sbs.softBodies[i];

                float totalMass = 0;
                float totalForce = 0;
                float totalFriction = 0;
                float totalVelocity = 0;
                float totalShearDisplacement = 0;
                float3 centerVel = 0;
                HashSet<int2> grids = new();
                HashSet<int2> neighborGrids = new();
                var bodyCollisions = softBody.collisions;
                if (bodyCollisions.Count == 0)
                    continue;

                foreach (var collision in bodyCollisions)
                {
                    if (!(collision.terrain == this)) continue;
                    sbs.GetParticleBodyAndIndex(collision.index, out _, out int localIndex);

                    // bool hasCollided = collision.lambda > 0;
                    float3 gridPoint = WorldPos2Grid((float3)collision.q);
                    int2 grid = new((int)gridPoint.x, (int)gridPoint.z);

                    float3 contactForce = (float3)collision.F / Simulation.get.substeps;
                    float3 normalForce = math.dot(new float3(0, 1, 0), (float3)contactForce) * new float3(0, 1, 0);
                    float3 frictionForce = (float3)contactForce - normalForce;
                    float3 effectiveVel = (float3)(softBody.particles[localIndex].vel * softBody.particles[localIndex].Mass);

                    grids.Add(grid);

                    centerVel += effectiveVel;
                    totalMass += (float)softBody.particles[localIndex].Mass;
                    totalForce += math.max(0, math.length(contactForce));
                    totalFriction += math.length(frictionForce);
                    totalVelocity += math.length((float3)(softBody.particles[localIndex].vel));
                    //totalShearDisplacement += (float)collision.shearDisplacement;

                    float3 contactDisplace = (float3)(softBody.particles[localIndex].pos - collision.q);
                    float3 contactDisplaceN = math.dot(contactDisplace, (float3)collision.N) * (float3)collision.N;
                    float3 contactDisplaceT = contactDisplace - contactDisplaceN;

                    totalShearDisplacement += math.length(contactDisplaceT) * (float)softBody.particles[localIndex].Mass;
                }

                if (grids.Count == 0)
                    continue;

                //gridPoints.UnionWith(grids);

                centerVel /= totalMass;
                totalVelocity /= bodyCollisions.Count;
                totalShearDisplacement /= totalMass;

                if (convexHull)
                    FillConvexHull(ComputeConvexHull(grids), grids);

                
                //convexGridPoints.UnionWith(grids);

                // Set respond time according to velocity;
                float bodyContactTime = maxContactTime / (1 + contactK * totalVelocity);
                bodyContactTime = minContactTime + (maxContactTime - minContactTime) * math.exp(-totalVelocity / contactK);
                float deformRate = dt / bodyContactTime;

                // Calculate terrain deformation per cell per frame
                float A_contact = areaCell * grids.Count;
                float gridPressure = totalForce / A_contact;
                float sinkage = (gridPressure) / Stiffness;
                if (sinkage > 1)
                    sinkage = math.pow(sinkage, 1 / groundMaterial.n);

                float shearDisplace = (float)totalShearDisplacement;
                float slipSinkage = GetSlipSinkage(sinkage, gridPressure, shearDisplace, A_contact);
                float sinkagePerFrame = 0;
                sinkagePerFrame += useSinkage ? sinkage : 0;
                sinkagePerFrame += useSlipSinkage ? slipSinkage : 0;
                sinkagePerFrame *= deformRate;
                foreach (var grid in grids)
                {
                    if (!gridDisplacements.ContainsKey(grid))
                        gridDisplacements.Add(grid, 0);
                    gridDisplacements[grid] -= sinkagePerFrame;
                }


                // Calculate terrain vertical accumulation per cell per frame
                // V1 - V1c = A_contact * dL * (2 * poissonR)
                float poissonR = groundMaterial.PoissonRatio;
                float lateralBump = (2 * poissonR) * sinkagePerFrame * A_contact / (areaCell * neighborGrids.Count);
                float lateralBumpPerCellPerFrame = lateralBump;
                float lateralBumpTotalHeights = (2 * poissonR) * sinkagePerFrame * A_contact / areaCell;

                Timer bumpTimer = new();
                bumpTimer.Tic();
                GetBumpDisplacement(grids, centerVel, lateralBumpTotalHeights);
                bumpTimer.Toc();

                bumpFrames++;
                bumpTime += bumpTimer.Duration() * 0.001f;
                bumpTimeAvg = bumpTime / bumpFrames;

                plotTotalForce += totalForce;
                plotTotalArea += A_contact;
                plotTotalSinkage += sinkage * A_contact;
                plotTotalSlipSinkage += slipSinkage * A_contact;
                plotTotalShearDisplace += shearDisplace;

            }


            if (gridDisplacements.Count <= 0)
            {
                return;
            }
            if (printConvexHull)
            {
                printConvexHull = false;

                GridsToTexture(gridPoints, "1");
                GridsToTexture(convexGridPoints, "2");
            }

            Simulation.get.totalForce += plotTotalForce;
            Simulation.get.totalArea += plotTotalArea;
            if (plotTotalArea == 0)
                Simulation.get.totalPressure += 0;
            else
                Simulation.get.totalPressure += plotTotalForce / plotTotalArea;

            Simulation.get.totalSinkage += plotTotalSinkage;
            Simulation.get.totalSlipSinkage += plotTotalSlipSinkage;
            Simulation.get.totalShearDisplace += plotTotalShearDisplace;

            void GetBumpDisplacement(HashSet<int2> grids, float3 centerVel, float bumpVolume)
            {
                if (!printLaterelBumps)
                    return;

                float totalWeight = 0;
                Dictionary<int2, float> neighborGridWeights = new();

                // Get local search area
                (int2 localMin, int2 localMax) = grids.Aggregate(
                    (localMin: grids.First(), localMax: grids.First()),
                    (acc, next) => (math.min(acc.localMin, next), math.max(acc.localMax, next))
                );
                int2 centerGrid = grids.Aggregate(int2.zero, (acc, next) => acc + next) / grids.Count;

                float3 velDir = math.normalize(centerVel);
                bumpOffset = neighbourSearch;
                // Search neighbour bump grid
                for (int zi = localMin.y - bumpOffset; zi <= localMax.y + bumpOffset; ++zi)
                {
                    for (int xi = localMin.x - bumpOffset; xi <= localMax.x + bumpOffset; ++xi)
                    {
                        int2 bumpGrid = new(xi, zi);
                        float3 gridDir = (bumpGrid - centerGrid).xxy;
                        gridDir.y = 0;
                        float distance = math.length(gridDir);
                        gridDir = math.normalizesafe(gridDir, 0);
                        // If grid has no displacement, it is a potential bump grid
                        if (!grids.Contains(bumpGrid))
                        {
                            // B. Only checking adjacent cells - increasing this would allow increasing the area of the bump
                            for (int zi_sub = -neighbourSearch; zi_sub <= neighbourSearch; zi_sub++)
                            {
                                for (int xi_sub = -neighbourSearch; xi_sub <= neighbourSearch; xi_sub++)
                                {
                                    int2 subGrid = bumpGrid + new int2(xi_sub, zi_sub);

                                    // Skip if same grid
                                    if (xi_sub + zi_sub == 0)
                                        continue;

                                    // C. If there is a contact point around the cell
                                    if (grids.Contains(subGrid))
                                    {
                                        float weight = 1;
                                        if (directionBump)
                                        {
                                            //float distance = Mathf.Sqrt(xi_sub * xi_sub + zi_sub * zi_sub);
                                            float offset = 2;
                                            float dist_offset = math.max(0, distance - offset);
                                            dist_offset = Mathf.Sqrt(xi_sub * xi_sub + zi_sub * zi_sub);
                                            float sigma = (neighbourSearch + 1) / 2.447f;
                                            float distanceWeight = Mathf.Exp(-dist_offset * dist_offset / (2 * sigma * sigma));
                                            //distanceWeight = 1;
                                            //gridDir = math.normalizesafe(new float3(-xi_sub, 0, zi_sub), 0);
                                            float alignWeight = math.dot(velDir, gridDir);
                                            alignWeight = math.max(alignWeight, 0);
                                            weight = distanceWeight * alignWeight;
                                            //totalWeight += weight;
                                        }

                                        if (neighborGridWeights.ContainsKey(bumpGrid))
                                        {
                                            if(weight < neighborGridWeights[bumpGrid])
                                                neighborGridWeights[bumpGrid] = weight;
                                        }
                                        else
                                        {
                                            neighborGridWeights.Add(bumpGrid, weight);
                                        }
                                        //break;
                                    }
                                }
                                //if (neighborGridWeights.ContainsKey(bumpGrid))
                                //    break;
                            }
                        }
                    }
                }
                totalWeight = neighborGridWeights.Values.Sum(x => x);
                foreach (var neighbourGrid in neighborGridWeights)
                {
                    if (!gridDisplacements.ContainsKey(neighbourGrid.Key))
                        gridDisplacements.Add(neighbourGrid.Key, 0);

                    float lateralBumpPerCellPerFrame = bumpVolume * neighbourGrid.Value / totalWeight;
                    gridDisplacements[neighbourGrid.Key] += lateralBumpPerCellPerFrame;
                }
            }
            void GetBumpDisplacement2(HashSet<int2> grids, float3 centerVel, float bumpVolume)
            {
                HashSet<int2> neighborGrids = new HashSet<int2>();
                Dictionary<int2, float> distanceMap = new Dictionary<int2, float>();
                Queue<(int2 cell, float dist)> queue = new Queue<(int2, float)>();

                foreach (var grid in grids)
                {
                    queue.Enqueue((grid, 0f));
                    distanceMap[grid] = 0f;
                }

                int2[] directions = new int2[]
                {
                    new int2(1,0), new int2(-1,0), new int2(0,1), new int2(0,-1),
                    new int2(1,1), new int2(-1,-1), new int2(1,-1), new int2(-1,1)
                };
                
                while (queue.Count > 0)
                {
                    var (current, dist) = queue.Dequeue();

                    foreach (var dir in directions)
                    {
                        int2 neighbor = current + dir;
                        float newDist = math.distance((float2)neighbor, (float2)current); // or accumulate

                        if (grids.Contains(neighbor)) continue; // skip original collision
                        if (newDist > neighbourSearch) continue;

                        if (!distanceMap.ContainsKey(neighbor) || newDist < distanceMap[neighbor])
                        {
                            distanceMap[neighbor] = newDist;
                            neighborGrids.Add(neighbor);
                            queue.Enqueue((neighbor, newDist));
                        }
                    }
                }
            }
            float GetSlipSinkage(float sinkage, float pressure, float shearDisplace, float contactArea)
            {
                float zj = 0;

                float cohesion = groundMaterial.cohesion * groundMaterial.Nc;
                float surcharge = groundMaterial.Nq * sinkage;
                float weight = 0.5f * groundMaterial.Nr * Mathf.Sqrt(contactArea);
                float maxShear = GetMaxShear(pressure);

                float capacity = cohesion + (surcharge + weight) * groundMaterial.unitWeight;
                //Debug.Log($"{cohesion} {surcharge} {weight} {capacity}");

                zj = shearDisplace * (pressure - cohesion - (surcharge + weight) * groundMaterial.unitWeight)
                    / (maxShear + groundMaterial.unitWeight * groundMaterial.Nq * shearDisplace);
                zj = math.max(0, zj);
                return zj;
            }
            List<int2> ComputeConvexHull(HashSet<int2> points)
            {
                if (points == null) return new List<int2>();
                if (points.Count < 3) return new List<int2>(points);

                // **步驟 1：找到最低的點（最左下角）**
                int2 pivot = points.OrderBy(p => p.y).ThenBy(p => p.x).First();

                // **步驟 2：根據極角排序點**
                List<int2> sortedPoints = points.OrderBy(p =>
                {
                    double angle = Math.Atan2(p.y - pivot.y, p.x - pivot.x);
                    return angle;
                }).ToList();

                // **步驟 3：使用棧（Stack）構造凸包**
                Stack<int2> hull = new();
                hull.Push(sortedPoints[0]);
                hull.Push(sortedPoints[1]);

                for (int i = 2; i < sortedPoints.Count; i++)
                {
                    int2 top = hull.Pop();
                    while (hull.Count > 0 && CrossProduct(hull.Peek(), top, sortedPoints[i]) <= 0)
                    {
                        top = hull.Pop(); // 移除形成內角的點
                    }
                    hull.Push(top);
                    hull.Push(sortedPoints[i]);
                }

                return hull.ToList();

                int CrossProduct(int2 A, int2 B, int2 C)
                {
                    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
                }
            }
            void FillConvexHull(List<int2> convexHull, HashSet<int2> filledPoints)
            {
                if (convexHull.Count == 0) return;
                // 找到 Y 軸的最小與最大範圍
                int minY = convexHull.Min(p => p.y);
                int maxY = convexHull.Max(p => p.y);

                for (int y = minY; y <= maxY; y++)
                {
                    List<int> intersections = new List<int>();

                    for (int i = 0; i < convexHull.Count; i++)
                    {
                        int2 p1 = convexHull[i];
                        int2 p2 = convexHull[(i + 1) % convexHull.Count];

                        if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) // 找 Y 軸交點
                        {
                            int x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                            intersections.Add(x);
                        }
                    }

                    intersections.Sort();

                    // 逐條掃描填充區域
                    for (int i = 0; i < intersections.Count; i += 2)
                    {
                        int xStart = intersections[i];
                        int xEnd = intersections[i + 1];

                        for (int x = xStart; x <= xEnd; x++)
                        {
                            filledPoints.Add(new int2(x, y)); // 填充點
                        }
                    }
                }
            }

            void GridsToTexture(HashSet<int2> gridPoints, string name = "")
            {

                var gridHeights = gridPoints;
                if (gridHeights.Count == 0)
                {
                    m_brushUV = 0f;
                    m_BrushSize = 0f;
                    m_BrushOpacity = 0f;
                    return;
                }
                (int2 localMin, int2 localMax) = gridHeights.Aggregate(
                    (localMin: gridHeights.First(), localMax: gridHeights.First()),
                    (acc, next) => (math.min(acc.localMin, next), math.max(acc.localMax, next))
                );
                int2 centerGrid = (localMin + localMax) / 2;
                int brushResolution = math.cmax(localMax - localMin);
                brushResolution += (brushResolution % 2) == 1 ? 4 : 3;
                brushResolution = math.max(65, brushResolution);

                //brushTexture.Reinitialize(brushResolution, brushResolution);
                Texture2D template = new Texture2D(brushResolution, brushResolution, TextureFormat.RGB24, -1, false);

                Color32[] colors = new Color32[brushResolution * brushResolution];
                //float maxDisplacement = gridHeights.Max();
                //maxDisplacement = gridHeights.Aggregate(
                //    (max, next) =>
                //    (math.abs(max) < math.abs(next) ? math.abs(next) : math.abs(max))
                //);
                foreach (var height in gridHeights)
                {
                    int2 grid0 = centerGrid - brushResolution / 2;
                    int2 uv = height - grid0;

                    Color32 maxColor = Color.red;
                    Color32 black = Color.black;
                    colors[uv.x + uv.y * brushResolution] = Color.red;
                }
                template.SetPixels32(colors);
                template.Apply();

                byte[] mapBytes = ImageConversion.EncodeToPNG(template);

                string fileName = System.DateTime.Now.ToString("MMddyyyy_hh_mm_ss");
                File.WriteAllBytes(Application.dataPath + "/./Textures/ConvexHull/" + fileName + name + ".png", mapBytes);

                UnityEngine.Object.Destroy(template);
#if UNITY_EDITOR
                UnityEditor.AssetDatabase.Refresh();
#endif
                return;
            }
        }

        #endregion
        #region Terrain Data Getters
        public bool RayCast(Ray ray, out RaycastHit hit, float maxDist)
        {
            return m_Collider.Raycast(ray, out hit, maxDist);
        }
        public void World2UV(float3 pos, out float x, out float y)
        {
            x = pos.x - transform.position.x;
            y = pos.z - transform.position.z;

            x /= terrainWidth;
            y /= terrainLength;
        }
        public float SampleHeight(float3 pos)
        {
            World2UV(pos, out float x, out float y);

            if (x < 0 || x > 1 || y < 0 || y > 1)
                return float.NegativeInfinity;
            return terrainData.GetInterpolatedHeight(x, y);
        }

        public float3 SampleNormal(float3 pos)
        {
            World2UV(pos, out float x, out float y);

            return terrainData.GetInterpolatedNormal(x, y);
        }

        public float3 World2AlphaGrid(float3 worldPos)
        {
            float3 alphamapScale = 0;
            alphamapScale.x = terrainWidth / (terrainData.alphamapWidth - 1);
            alphamapScale.z = terrainLength / (terrainData.alphamapHeight - 1);

            float3 localPos = worldPos - (float3)transform.position;
            return new float3(localPos.x / alphamapScale.x,
                              localPos.y,
                              localPos.z / alphamapScale.z);
        }
        public float3 WorldPos2Grid(float3 worldPos)
        {
            float3 localPos = worldPos - (float3)transform.position;
            localPos /= terrainData.heightmapScale;

            return math.floor(localPos);
        }

        public float3 Grid2WorldGrid(float3 grid)
        {
            return new float3(grid.x * terrainData.heightmapScale.x,
                              grid.y,
                              grid.z * terrainData.heightmapScale.z);
        }

        public float GetMaxShear(float pressure)
        {
            return groundMaterial.cohesion + pressure * math.tan(groundMaterial.frictionAngleInRadian);
        }

        public float GetShear(float maxShear, float displace)
        {
            return maxShear * (1 - MathF.Exp(-displace / groundMaterial.K));
        }

        public float GetShearDisplace(float pressure, float shear)
        {
            float shearRatio = math.max(1, shear / GetMaxShear(pressure));
            Debug.Log(shearRatio);
            return -Mathf.Log(1 - shearRatio) * groundMaterial.K;
        }

        public float Stiffness
        {
            get
            {
                float k_c = groundMaterial.k_c / CellWidth;
                float k_phi = groundMaterial.k_phi;
                return (k_c + k_phi);
            }
        }
        public float Pressure(float sinkage)
        {
            return Stiffness * math.pow(sinkage, groundMaterial.n);
        }
        #endregion

        #region MonoBehaviour
        private void Start()
        {
            Terrain = GetComponent<Terrain>();
            terrainData = Terrain.terrainData;
            m_Collider = GetComponent<TerrainCollider>();

            heightmapHeight = heightmapWidth = terrainData.heightmapResolution;

            terrainWidth = terrainData.size.x;
            terrainLength = terrainData.size.z;
            terrainHeight = terrainData.size.y;

            //float2 key = new(transform.position.x / terrainWidth, transform.position.z / terrainLength);
            //Simulation.get.AddTerrain(this, (int2)math.floor(key));
            Simulation.get.terrainSystem.AddTerrain(this);
            // Save initial height for undo after simulation
            originalHeightmap = new RenderTexture(terrainData.heightmapTexture);
            Graphics.Blit(terrainData.heightmapTexture, originalHeightmap);

            footprintMat = new Material(Shader.Find("Custom/FootPrint"));

            coroutineTimer.TicAndPause();
        }
        private void Update()
        {
            StartCoroutine(PaintFootPrint());
        }
        private void OnDisable()
        {
            RenderTexture.active = originalHeightmap;
            terrainData.CopyActiveRenderTextureToHeightmap(
                new RectInt(0, 0, heightmapWidth, heightmapHeight),
                Vector2Int.zero,
                TerrainHeightmapSyncControl.HeightAndLod);
            RenderTexture.active = null;
        }
        #endregion
    }
}