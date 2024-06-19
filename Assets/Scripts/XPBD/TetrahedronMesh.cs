using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

[CreateAssetMenu(fileName = "NewTetrahedronMesh", menuName = "TetrahedronMesh")]
public class TetrahedronMesh : ScriptableObject
{
    public Vector3[] vertices;
    public int[] faces;
    public int[] tets;
    public int[] edges;

    public int[] GraphColoring()
    {
        var adjacencyList = BuildAdjacencyList();

        int n = adjacencyList.Count;

        int[] colors = new int[n];
        bool[] available = new bool[n];

        for (int i = 0; i < n; i++)
            colors[i] = -1;

        colors[0] = 0;

        for (int i = 1; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                available[j] = true;

            foreach (int neighbor in adjacencyList[i])
            {
                if (colors[neighbor] != -1)
                    available[colors[neighbor]] = false;
            }

            int cr;
            for (cr = 0; cr < n; cr++)
            {
                if (available[cr])
                    break;
            }

            colors[i] = cr;
        }

        return colors;
    }

    private Dictionary<int, List<int>> BuildAdjacencyList()
    {
        Dictionary<int, List<int>> adjacencyList = new Dictionary<int, List<int>>();

        // 遍歷所有四面體
        for (int i = 0; i < tets.Length / 4; i++)
        {
            int4 tet1 = new (tets[4 * i], tets[4 * i + 1], tets[4 * i + 2], tets[4 * i + 3]);
            for (int j = i + 1; j < tets.Length / 4; j++)
            {
                int4 tet2 = new(tets[4 * j], tets[4 * j + 1], tets[4 * j + 2], tets[4 * j + 3]);

                // 判斷是否相鄰
                if (AreAdjacent(tet1, tet2))
                {
                    if (!adjacencyList.ContainsKey(i))
                        adjacencyList[i] = new List<int>();

                    if (!adjacencyList.ContainsKey(j))
                        adjacencyList[j] = new List<int>();

                    // 記錄相鄰關係
                    adjacencyList[i].Add(j);
                    adjacencyList[j].Add(i);
                }
            }
        }

        return adjacencyList;
    }

    private bool AreAdjacent(int4 t1, int4 t2)
    {
        int sharedVertices = 0;
        // 檢查每個點的相等性
        for(int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (t1[i] == t2[j])
                    sharedVertices++;

                if(sharedVertices >= 1)
                    return true;
            }
        }
        return false;
    }
}
