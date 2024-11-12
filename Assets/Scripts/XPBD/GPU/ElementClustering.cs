// Implementation of [Parallel Block Neo-Hookean XPBD using Graph Clustering]
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEditor.Experimental.AssetDatabaseExperimental.AssetDatabaseCounters;
using static UnityEngine.Rendering.DebugUI;

namespace XPBD
{
    public class ElementClustering
    {
        public class Cluster
        {
            public int color = -1;
            public List<int> constraints;

            public Cluster()
            {
                color = -1;
                constraints = new List<int>();
            }
        }
        int Ks; // Maximum cluster size
        Dictionary<int, List<int>> graph; // Graph adjacency list
        Dictionary<int, int> parentMap; // £k map
        List<Cluster> clusters; // V¡Â

        ElementConstraint[] elementConstraints;

        public ElementClustering(int maxClusterSize, ElementConstraint[] constraints)
        {
            Ks = maxClusterSize;
            elementConstraints = constraints;

            graph = BuildAdjacencyList();
            parentMap = new();
            clusters = new List<Cluster>();

            foreach(var key in graph.Keys)
            {
                parentMap[key] = -1;
            }
        }

        public List<Cluster> PerformClusteringAndColoring()
        {
            while (parentMap.Any(pair => pair.Value == -1))
            {
                int clusterSize = Ks;
                while (clusterSize > 0)
                {
                    int u0 = SelectedSeedVertex();

                    if (u0 == -1) break;

                    Queue<int> bfsQueue = new();
                    bfsQueue.Enqueue(u0);

                    HashSet<int> visited = new() { u0 };

                    //List<int> currentCluster = new List<int>() { u0 };
                    //parentMap[u0] = clusters.Count;

                    // Do BFS from u0
                    while (bfsQueue.Count > 0)
                    {
                        int j = bfsQueue.Dequeue();

                        if (parentMap[j] != -1)
                            continue;

                        //List<int> currentCluster = new List<int>() { j };
                        Cluster currentCluster = new();
                        currentCluster.constraints.Add(j);
                        parentMap[j] = clusters.Count;

                        var neighbors = graph[j].Where(k => parentMap[k] == -1)
                                                 .OrderBy(k => elementConstraints[j].Distance(elementConstraints[k]))
                                                 .ToList();

                        foreach (int k in neighbors)
                        {
                            if (!visited.Contains(k))
                            {
                                bfsQueue.Enqueue(k);
                                visited.Add(k);
                            }

                            if (currentCluster.constraints.Count < clusterSize)
                            {
                                currentCluster.constraints.Add(k);
                                parentMap[k] = clusters.Count; // Assign £k(k)
                                                               //bfsQueue.Enqueue(k);
                            }
                        }

                        if (currentCluster.constraints.Count >= clusterSize)
                        {
                            clusters.Add(currentCluster); // Add to V¡Â
                        }
                        else if (currentCluster.constraints.Count < clusterSize)
                        {
                            // Undo the cluster if it is too small
                            foreach (int k in currentCluster.constraints)
                            {
                                parentMap[k] = -1; // Unassign £k(k)
                            }
                            currentCluster.constraints.Clear();
                        }
                    }
                    clusterSize--;
                }
            }

            //Coloring();
            return Coloring();
        }

        public List<Cluster> Coloring()
        {
            var adjacencyList = BuildAdjacencyClusterList();

            //List<int> colors = new();
            int N = clusters.Count;

            if(N == 0)
                return clusters;

            //colors = Enumerable.Repeat(-1, N).ToList();
            bool[] available = new bool[N];

            clusters[0].color = 0;

            for (int i = 1; i < N; i++)
            {
                Cluster cluster = clusters[i];
                for (int j = 0; j < N; j++)
                    available[j] = true;

                foreach (var neighbor in adjacencyList[cluster])
                {
                    if (neighbor.color != -1)
                        available[neighbor.color] = false;
                }

                int cr;
                for (cr = 0; cr < N; cr++)
                {
                    if (available[cr])
                        break;
                }

                cluster.color = cr;
            }

            return clusters;
        }
        private Dictionary<int, List<int>> BuildAdjacencyList()
        {
            Dictionary<int, List<int>> adjacencyList = new Dictionary<int, List<int>>();
            //System.Array.Fill(adjacencyList, new List<int>());
            for (int i = 0; i < elementConstraints.Length; i++)
            {
                if (!elementConstraints[i].active)
                    continue;

                if(!adjacencyList.ContainsKey(i))
                    adjacencyList[i] = new List<int>();
                for (int j = i + 1; j < elementConstraints.Length; j++)
                {
                    //int4 tet2 = elementConstraints[j].tet;
                    // check adjacent
                    if (!elementConstraints[j].active)
                        continue;

                    if (!adjacencyList.ContainsKey(j))
                        adjacencyList[j] = new List<int>();

                    if (elementConstraints[i].AreAdjacent(elementConstraints[j]))
                    {
                        // record adjacency

                        //adjacencyList[i] ??= new List<int>();
                        //adjacencyList[j] ??= new List<int>();

                        adjacencyList[i].Add(j);
                        adjacencyList[j].Add(i);
                    }
                }
            }
            return adjacencyList;
        }

        // Select the constraint with the minimum distance to all neighbor constraint
        // Implementation of Equation 16
        private int SelectedSeedVertex()
        {
            int minVertex = -1;
            float minSum = float.MaxValue;

            foreach (var j in graph.Keys)
            {
                if (parentMap[j] != -1)
                    continue;

                float sum = 0;

                // Sum all neighbor distance
                foreach (int neighbor in graph[j])
                {
                    if (parentMap[neighbor] == -1)
                    {
                        sum += elementConstraints[j].Distance(elementConstraints[neighbor]);
                    }
                }

                // Check if distance sum is smaller than min
                if (sum < minSum)
                {
                    minSum = sum;
                    minVertex = j;
                }
            }

            return minVertex;
        }

        private Dictionary<Cluster, List<Cluster>> BuildAdjacencyClusterList()
        {
            if(clusters.Count == 0)
                return null;

            Dictionary<Cluster, List<Cluster>> adjacencyList = new();
            for (int i = 0; i < clusters.Count; i++)
            {
                Cluster cluster1 = clusters[i];
                for (int j = i + 1; j < clusters.Count; j++)
                {
                    Cluster cluster2 = clusters[j];
                    // check adjacent
                    if (ClustersAdjeacent(i, j))
                    {
                        // record adjacency
                        if (!adjacencyList.ContainsKey(cluster1))
                            adjacencyList[cluster1] = new List<Cluster>();
                        if (!adjacencyList.ContainsKey(cluster2))
                            adjacencyList[cluster2] = new List<Cluster>();
                        adjacencyList[cluster1].Add(cluster2);
                        adjacencyList[cluster2].Add(cluster1);
                    }
                }
            }
            return adjacencyList;
        }

        private bool ClustersAdjeacent(int i, int j)
        {
            var cluster1 = clusters[i];
            var cluster2 = clusters[j];

            foreach (var c1 in cluster1.constraints)
            {
                foreach (var c2 in cluster2.constraints)
                {
                    if (elementConstraints[c1].AreAdjacent(elementConstraints[c2]))
                        return true;
                }
            }

            return false;
        }
    }

}