using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
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
    public class Hash
    {
        REAL spacing;
        int tableSize;
        int[] cellStart;
        int[] cellEntries;
        public int[] queryIds;
        public int querySize { get; private set; }
        public Hash(REAL spacing, int maxNumObjects)
        {
            this.spacing = spacing;
            tableSize = 2 * maxNumObjects;
            cellStart = new int[tableSize + 1];
            cellEntries = new int[maxNumObjects];
            queryIds = new int[maxNumObjects];
            querySize = 0;
        }

        public void Create(REAL3[] pos)
        {
            int numObjects = Mathf.Min(pos.Length, cellEntries.Length);

            // determine cell sizes
            Array.Fill(cellStart, 0);
            Array.Fill(cellEntries, 0);

            for(int i = 0; i < numObjects; ++i)
            {
                int h = hashPos(pos[i]);
                cellStart[h]++;
            }

            // determine cells starts
            int start = 0;
            for (int i = 0; i < tableSize; ++i)
            {
                start += cellStart[i];
                cellStart[i] = start;
            }
            cellStart[tableSize] = start;

            // fill in objects ids
            for (int i = 0; i < numObjects; ++i)
            {
                int h = hashPos(pos[i]);
                cellStart[h]--;
                cellEntries[cellStart[h]] = i;
            }
        }

        public void Query(REAL3 pos, REAL maxDist)
        {
            int x0 = intCoord(pos[0] - maxDist);
            int y0 = intCoord(pos[1] - maxDist);
            int z0 = intCoord(pos[2] - maxDist);

            int x1 = intCoord(pos[0] + maxDist);
            int y1 = intCoord(pos[1] + maxDist);
            int z1 = intCoord(pos[2] + maxDist);

            querySize = 0;

            for (int xi = x0; xi <= x1; xi++)
            {
                for (int yi = y0; yi <= y1; yi++)
                {
                    for (int zi = z0; zi <= z1; zi++)
                    {
                        int h = hashCoords(xi, yi, zi);
                        var start = cellStart[h];
                        var end = cellStart[h + 1];

                        for (var i = start; i < end; i++)
                        {
                            queryIds[querySize] = cellEntries[i];
                            querySize++;

                            if (querySize >= queryIds.Length)
                                Array.Resize(ref queryIds, 2 * queryIds.Length);
                        }
                    }
                }
            }
        }

        private int hashPos(REAL3 pos)
        {
            return this.hashCoords(
                this.intCoord(pos[0]),
                this.intCoord(pos[1]),
                this.intCoord(pos[2]));
        }

        private int intCoord(REAL coord)
        {
            return (int)math.floor(coord / this.spacing);
        }

        private int hashCoords(int xi, int yi, int zi)
        {
            int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);  // fantasy function
            return Mathf.Abs(h) % this.tableSize;
        }
    }
}
