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

        [DebugOnly,SerializeField, Min(0)]private float m_BrushOpacity = 0.1f;
        [DebugOnly, SerializeField] private float m_BrushSize = 0.01f;
        float2 m_brushUV;
        public Texture2D brushTexture;

        [SerializeField] private RenderTexture originalHeightmap;
        [SerializeField] float maxContactTime = 10f;
        [SerializeField] float minContactTime = 0.05f;
        [SerializeField] float contactK = 0.1f;
        [SerializeField] float L_0 = 0.3f;
        [SerializeField] int neighbourSearch = 2;
        [SerializeField] int bumpOffset = 5;

        [Header("Footprint Settings")]
        [SerializeField] bool useSinkage = true;
        [SerializeField] bool useSlipSinkage = true;
        [SerializeField] bool printFootPrint = true;
        [SerializeField] bool printLaterelBumps = true;
        [SerializeField] Material footprintMat;

        public GroundMaterial GroundMaterial => groundMaterial;
        [SerializeField] GroundMaterial groundMaterial;

        [Header("Gaussian Filter")]
        [SerializeField]
        bool filter = true;
        [SerializeField] Material gaussianMaterial;

        public List<MyCollision> collisions = new List<MyCollision>();

        [DebugOnly] public float maxOpacity = 0;

        [DebugOnly] public float averageOpacity = 0;
        int times = 0;
        #region Terrain Paint
        private IEnumerator PaintFootPrint()
        {
            // Get the current BrushTransform under the mouse position relative to the Terrain
            BrushTransform brushXform = TerrainPaintUtility.CalculateBrushTransform(Terrain, m_brushUV, m_BrushSize, 0);

            // Get the PaintContext for the current BrushTransform. This has a sourceRenderTexture from which to read existing Terrain texture data
            // and a destinationRenderTexture into which to write new Terrain texture data
            PaintContext paintContext = TerrainPaintUtility.BeginPaintHeightmap(Terrain, brushXform.GetBrushXYBounds());
            //PaintContext controlContext = GatherAlphaMap(brushXform);
            // Get the built-in painting Material reference
            Material mat;// = TerrainPaintUtility.GetBuiltinPaintMaterial();
            mat = footprintMat;
            // Bind the current brush texture
            mat.SetTexture("_BrushTex", brushTexture);
            //mat.SetTexture("_Control", controlContext.sourceRenderTexture);
            // Bind the tool-specific shader properties
            //mat.SetVector("groundMaterial0", groundMaterialData0.xyxx);

            var opacity = m_BrushOpacity;
            //opacity /= (terrainData.heightmapScale.y);
            //opacity *= PaintContext.kNormalizedHeightScale;
            mat.SetVector("_BrushParams", new Vector4(opacity, 0.0f, 0.0f, 0.0f));
            mat.SetFloat("terrainHeight", terrainHeight);
            // Setup the material for reading from/writing into the PaintContext texture data. This is a necessary step to setup the correct shader properties for appropriately transforming UVs and sampling textures within the shader
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
            //TerrainPaintUtility.ReleaseContextResources(controlContext);

            yield return null;

            PaintContext GatherAlphaMap(BrushTransform brushTransform)
            {
                Material copyTerrainLayerMaterial = footprintMat;

                int alphaMapResolution = terrainData.alphamapResolution;
                PaintContext ctx = PaintContext.CreateFromBounds(Terrain, brushTransform.GetBrushXYBounds(), alphaMapResolution, alphaMapResolution);
                ctx.CreateRenderTargets(RenderTextureFormat.ARGB32);

                ctx.Gather(
                    t => terrainData.alphamapTextures[0],
                    new Color(0, 0, 0, 0),
                    copyTerrainLayerMaterial, 1);
                return ctx;
            }
        }
        public void PaintFootPrints(SoftBodySystem sbs, float dt)
        {
            if (!printFootPrint)
                return;
            // Create brush texture, Red: Lower, Green: Raise
            brushTexture = brushTexture != null ? brushTexture : new Texture2D(1,1, TextureFormat.RG16, -1, false);
            int bodyCount = sbs.softBodies.Count;
            if( bodyCount == 0 )
            {
                Debug.Log("No soft bodies");
                return;
            }

            // The min and max grid index
            int2 minGrid = new(heightmapWidth, heightmapHeight);
            int2 maxGrid = new(-1, -1);

            Dictionary<int2, float> gridDisplacements = new();
            Dictionary<int2, float> gridBumpDisplacements = new();
            // grid cell area
            float lenghtCellX = CellWidth;
            float lenghtCellZ = CellWidth;
            float areaCell = CellArea;

            for (int i = 0; i < bodyCount; i++)
            {
                SoftBody softBody = sbs.softBodies[i];

                float totalForce = 0;
                float totalFriction = 0;
                float totalVelocity = 0;
                float totalShearDisplacement = 0;
                HashSet<int2> grids = new();
                HashSet<int2> neighborGrids = new();

                var bodyCollisions = softBody.collisions;
                if (bodyCollisions.Count == 0)
                    continue;

                foreach(var collision in bodyCollisions)
                {
                    if(!(collision.terrain == this)) continue;
                    sbs.GetParticleBodyAndIndex(collision.index, out _, out int localIndex);

                    // bool hasCollided = collision.lambda > 0;
                    float3 gridPoint = WorldPos2Grid((float3)collision.q);
                    int2 grid = new((int)gridPoint.x, (int)gridPoint.z);

                    float3 normalForce = math.dot(new float3(0, 1, 0), (float3)collision.F) * new float3(0, 1, 0);
                    float3 frictionForce = (float3)collision.F - normalForce;
                    float3 effectiveVel = (float3)softBody.particles[localIndex].vel;

                    grids.Add(grid);
                    totalForce += math.max(0,math.dot(new float3(0, 1, 0), (float3)collision.F));
                    totalFriction += math.length(frictionForce);
                    totalVelocity += math.length((float3)(softBody.particles[localIndex].vel));
                    totalShearDisplacement += (float)collision.shearDisplacement;
                    //float F_weight = Simulation.get.gravity.y

                    minGrid = math.min(minGrid, grid);
                    maxGrid = math.max(maxGrid, grid);
                }

                if(grids.Count == 0)
                    continue;

                totalVelocity /= bodyCollisions.Count;
                FillConvexHull(ComputeConvexHull(grids), grids);
                // Calculate bump

                // Get local search area
                (int2 localMin, int2 localMax) = grids.Aggregate(
                    (localMin: maxGrid, localMax: minGrid),
                    (acc, next) => (math.min(acc.localMin, next), math.max(acc.localMax, next))
                );
                // Search neighbour bump grid
                for (int zi = localMin.y - bumpOffset; zi <= localMax.y + bumpOffset; ++zi)
                {
                    for(int xi = localMin.x - bumpOffset; xi <= localMax.x + bumpOffset; ++xi)
                    {
                        int2 bumpGrid = new (xi, zi);

                        // If grid has no displacement, it is a potential bump grid
                        if(!grids.Contains(bumpGrid))
                        {
                            // B. Only checking adjacent cells - increasing this would allow increasing the area of the bump
                            for (int zi_sub = -neighbourSearch; zi_sub <= neighbourSearch; zi_sub++)
                            {
                                for (int xi_sub = -neighbourSearch; xi_sub <= neighbourSearch; xi_sub++)
                                {
                                    int2 subGrid = bumpGrid + new int2(xi_sub, zi_sub);
                                    bool sameGrid = xi_sub + zi_sub == 0;
                                    // C. If there is a contact point around the cell
                                    if (grids.Contains(subGrid) && !sameGrid)
                                    {
                                        neighborGrids.Add(bumpGrid);
                                        break;
                                    }
                                }
                                if (neighborGrids.Contains(bumpGrid))
                                    break;
                            }
                        }
                    }
                }

                // Set respond time according to velocity;
                float bodyContactTime = maxContactTime / (1 + contactK * totalVelocity);
                bodyContactTime = minContactTime + (maxContactTime - minContactTime) * math.exp(-totalVelocity / contactK);
                float deformRate = dt / bodyContactTime;

                // Calculate terrain deformation per cell per frame
                // dL = gridPressure * L0 / youngModulus
                float youngModulus = Mathf.Max(groundMaterial.YoungModulus, 1e-3f);
                float A_contact = areaCell * grids.Count;
                float gridPressure = totalForce / (areaCell * grids.Count);
                float gridShear = totalFriction / (areaCell * grids.Count);
                float sinkage = gridPressure * L_0 / youngModulus;

                sinkage = math.pow((gridPressure) / Stiffness, 1 / groundMaterial.n);
                float shearDisplace = (float)totalShearDisplacement;
                float slipSinkage = GetSlipSinkage(sinkage, gridPressure, shearDisplace);
                float sinkagePerFrame = 0;
                sinkagePerFrame += (useSinkage) ? sinkage : 0;
                if (useSlipSinkage)
                    sinkagePerFrame += slipSinkage;

                // Calculate terrain vertical accumulation per cell per frame
                // V1 - V1c = A_contact * dL * (2 * poissonR)
                float poissonR = groundMaterial.PoissonRatio;
                float lateralBump = (2 * poissonR) * sinkagePerFrame * A_contact  / (areaCell * neighborGrids.Count);
                float lateralBumpPerCellPerFrame = lateralBump * deformRate;
                foreach (var grid in grids)
                {
                    if (!gridDisplacements.ContainsKey(grid))
                        gridDisplacements.Add(grid, 0);
                    gridDisplacements[grid] -= sinkagePerFrame * deformRate;
                }
                foreach (var neighbourGrid in neighborGrids)
                {
                    if (!printLaterelBumps)
                        break;
                    if (!gridDisplacements.ContainsKey(neighbourGrid))
                        gridDisplacements.Add(neighbourGrid, 0);
                    gridDisplacements[neighbourGrid] += lateralBumpPerCellPerFrame;
                }

            }

            if (gridDisplacements.Count <= 0)
            {
                return;
            }
            GridDisplacements2BrushTexture(gridDisplacements, brushTexture, out m_brushUV, out m_BrushSize, out m_BrushOpacity);
            maxOpacity = math.max(maxOpacity, m_BrushOpacity);

            float totalOpacity = averageOpacity * times + m_BrushOpacity;
            averageOpacity = totalOpacity / ++times;
            StartCoroutine(PaintFootPrint());

            float GetSlipSinkage(float sinkage, float pressure, float shearDisplace)
            {
                float zj = 0;

                float cohesion = 1.3f * groundMaterial.cohesion * groundMaterial.Nc;
                float surcharge = groundMaterial.Nq * sinkage;
                float weight = 0.4f * groundMaterial.Nr * lenghtCellX;
                float maxShear = GetMaxShear(pressure);

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
                if(convexHull.Count == 0) return;
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
            void GridDisplacements2BrushTexture(
                Dictionary<int2, float> gridHeights, Texture2D texture, out float2 brushUV, out float brushSize, out float brushOpacity)
            {
                if(gridHeights.Count == 0)
                {
                    brushUV = 0f;
                    brushSize = 0f;
                    brushOpacity = 0f;
                    return;
                }
                (int2 localMin, int2 localMax) = gridHeights.Aggregate(
                    (localMin: maxGrid, localMax: minGrid),
                    (acc, next) => (math.min(acc.localMin, next.Key), math.max(acc.localMax, next.Key))
                );
                int2 centerGrid = (localMin + localMax) / 2;
                int brushResolution = math.cmax(localMax - localMin);
                brushResolution += (brushResolution % 2) == 1 ? 4 : 3;
                brushResolution = math.max(65, brushResolution);

                texture.Reinitialize(brushResolution, brushResolution);
                
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
                texture.SetPixels32(colors);
                texture.Apply();

                brushUV = ((float2)(localMin + localMax) / 2f / new float2((heightmapWidth - 1), (heightmapHeight - 1)));
                brushSize = Grid2WorldGrid(brushResolution).x;
                brushOpacity = maxDisplacement;

                return;
            }
        }
        public void PaintFootPrints2(SoftBodySystem sbs, float dt)
        {
            if (!printFootPrint)
                return;
            // Create brush texture, Red: Lower, Green: Raise
            brushTexture = brushTexture != null ? brushTexture : new Texture2D(1, 1, TextureFormat.RG16, -1, false);
            int bodyCount = sbs.softBodies.Count;
            if (bodyCount == 0)
            {
                Debug.Log("No soft bodies");
                return;
            }
            Dictionary<int2, float> gridDisplacements = new();
            // grid cell area
            float lengthCellX = terrainWidth / (heightmapWidth - 1);
            float lengthCellZ = terrainLength / (heightmapHeight - 1);
            float areaCell = lengthCellX * lengthCellZ;

            foreach (var collision in collisions)
            {
                sbs.GetParticleBodyAndIndex(collision.index, out SoftBody body, out int localIndex);

                float3 gridPoint = WorldPos2Grid((float3)collision.q);
                int2 grid = new((int)gridPoint.x, (int)gridPoint.z);

                float3 N = new float3(0, 1, 0);
                float3 normalForce = math.dot(new float3(0, 1, 0), (float3)collision.F) * new float3(0, 1, 0);
                float3 frictionForce = (float3)collision.F - normalForce;

                float totalVelocity = math.max(0, math.length((float3)(body.particles[localIndex].vel)));
                // Set respond time according to velocity;
                float bodyContactTime = minContactTime + (maxContactTime - minContactTime) * math.exp(-totalVelocity / contactK);
                bodyContactTime = math.lerp(minContactTime, maxContactTime, math.exp(-totalVelocity / contactK));
                float deformRate = dt / bodyContactTime;
                // Calculate terrain deformation per cell per frame
                // dL = gridPressure * L0 / youngModulus
                float youngModulus = Mathf.Max(groundMaterial.YoungModulus, 1e-3f);
                float A_contact = areaCell;
                float gridPressure = math.length(normalForce) / A_contact;
                float gridShear = math.length(frictionForce) / A_contact;
                float sinkage = gridPressure * L_0 / youngModulus;
                sinkage = math.pow((gridPressure) / Stiffness, 1 / groundMaterial.n);

                float shearDisplace = (float)collision.shearDisplacement;
                float slipSinkage = GetSlipSinkage(sinkage, gridPressure, shearDisplace);
                //slipSinkage = 0;
                float sinkagePerFrame = 0;
                sinkagePerFrame += (useSinkage) ? sinkage : 0; 
                if (useSlipSinkage)
                    sinkagePerFrame += slipSinkage;
                if (!gridDisplacements.ContainsKey(grid))
                    gridDisplacements.Add(grid,0);
                gridDisplacements[grid] -= sinkagePerFrame * deformRate;

                if(collision.index==0)
                {
                    Debug.Log($"shearDisplace {shearDisplace}");
                    Debug.Log($"slipSinkage {slipSinkage}");
                    Debug.Log($"sinkage {sinkage}");
                }
            }

            if (gridDisplacements.Count <= 0)
            {
                return;
            }

            GetBumbDisplacement();

            GridDisplacements2BrushTexture(gridDisplacements, brushTexture, out m_brushUV, out m_BrushSize, out m_BrushOpacity);
            
            StartCoroutine(PaintFootPrint());

            float GetSlipSinkage(float sinkage, float pressure, float shearDisplace)
            {
                float zj = 0;

                float cohesion = 1.3f * groundMaterial.cohesion * groundMaterial.Nc;
                float surcharge = groundMaterial.Nq * sinkage;
                float weight = 0.4f * groundMaterial.Nr * lengthCellX * groundMaterial.unitWeight;
                float maxShear = GetMaxShear(pressure);

                zj = shearDisplace * (pressure - cohesion - (surcharge + weight))
                    / (maxShear + shearDisplace * groundMaterial.Nq);
                zj = math.max(0, zj);
                return zj;
            }

            void GetBumbDisplacement()
            {
                if(!printLaterelBumps)
                    return;
                foreach (var grid in gridDisplacements.Keys.ToList())
                {
                    var value = gridDisplacements[grid];

                    int subGridSize = 2 * neighbourSearch + 1;
                    float[,] gridWeights = new float[subGridSize, subGridSize];
                    float totalWeight = 0f;
                    for (int xi = 0; xi < subGridSize; xi++)
                        for (int zi = 0; zi < subGridSize; zi++)
                        {
                            int2 dxdy = new int2(xi - neighbourSearch, zi - neighbourSearch);
                            float dist = math.length(dxdy);
                            gridWeights[xi, zi] = dist;
                            totalWeight += dist;
                        }

                    float[,] subGridDisplacements = new float[subGridSize, subGridSize];
                    for (int xi = -neighbourSearch; xi <= neighbourSearch; xi++)
                    {
                        for (int zi = -neighbourSearch; zi <= neighbourSearch; zi++)
                        {
                            int2 currentGrid = grid + new int2(xi, zi);

                            // Out of range exception
                            if (currentGrid.x < 0 || currentGrid.x >= heightmapWidth || currentGrid.y < 0 || currentGrid.y >= heightmapHeight)
                                continue;

                            int2 subGrid = new int2(xi, zi) + neighbourSearch;

                            gridDisplacements.TryGetValue(currentGrid, out float tryValue);
                            if (tryValue >= 0)
                                subGridDisplacements[subGrid.x, subGrid.y] = -value * gridWeights[subGrid.x, subGrid.y];
                            else
                                totalWeight -= gridWeights[subGrid.x, subGrid.y];
                        }
                    }
                    for (int xi = -neighbourSearch; xi <= neighbourSearch; xi++)
                    {
                        for (int zi = -neighbourSearch; zi <= neighbourSearch; zi++)
                        {
                            int2 currentGrid = grid + new int2(xi, zi);

                            // Out of range exception
                            if (currentGrid.x < 0 || currentGrid.x >= heightmapWidth || currentGrid.y < 0 || currentGrid.y >= heightmapHeight)
                                continue;

                            int2 subGrid = new int2(xi, zi) + neighbourSearch;

                            if (subGridDisplacements[subGrid.x, subGrid.y] > 0)
                            {
                                if (!gridDisplacements.ContainsKey(currentGrid))
                                    gridDisplacements.Add(currentGrid, 0);
                                gridDisplacements[currentGrid] += subGridDisplacements[subGrid.x, subGrid.y] / totalWeight;
                            }
                        }
                    }
                }
            }

            void GridDisplacements2BrushTexture(
                Dictionary<int2, float> gridHeights, Texture2D texture, out float2 brushUV, out float brushSize, out float brushOpacity)
            {
                if (gridHeights.Count == 0)
                {
                    brushUV = 0f;
                    brushSize = 0f;
                    brushOpacity = 0f;
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

                texture.Reinitialize(brushResolution, brushResolution);

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
                texture.SetPixels32(colors);
                texture.Apply();

                brushUV = ((float2)(localMin + localMax) / 2f / new float2((heightmapWidth - 1), (heightmapHeight - 1)));
                brushSize = Grid2WorldGrid(brushResolution).x;
                brushOpacity = maxDisplacement;

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

            if(x < 0 || x > 1 || y < 0 || y > 1)
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
            originalHeightmap =new RenderTexture(terrainData.heightmapTexture);
            Graphics.Blit(terrainData.heightmapTexture, originalHeightmap);

            footprintMat = new Material(Shader.Find("Custom/FootPrint"));
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

        private void OnDrawGizmos()
        {
            //if (terrainData == null)
            //    return;
            //float3 center = m_brushUV.xxy * terrainData.size + (float3)transform.position;
            //
            //center.y = 5f;
            //
            //float3 size = m_BrushSize;
            //size.y = 0.1f;
            //Gizmos.color = Color.red;
            //Gizmos.DrawCube(center, size);
        }
        #endregion
    }
}