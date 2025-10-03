using System.Collections.Generic;
using VRageMath;

namespace ImprovedAI.Pathfinding
{
    public struct Edge
    {
        public Vector3I Target { get; set; }
        public float Weight { get; set; }
        public Edge(Vector3I target, float weight)
        {
            Target = target;
            Weight = weight;
        }
    }
    public class AdjacencyList
    {
        private Dictionary<Vector3I, List<Edge>> adjacencyList;

        public AdjacencyList()
        {
            adjacencyList = new Dictionary<Vector3I, List<Edge>>();
        }

        /// <summary>
        /// Clear cached nodes.
        /// </summary>
        public void Clear()
        {
            adjacencyList.Clear();
        }

        /// <summary>
        /// Add a unidirectional edge from source to target
        /// </summary>
        public void AddEdge(Vector3I from, Vector3I to, float cost)
        {
            // Ensure the source node exists in the list
            if (!adjacencyList.ContainsKey(from))
            {
                adjacencyList[from] = new List<Edge>();
            }

            // Add the edge
            adjacencyList[from].Add(new Edge(to, cost));
        }
        /// <summary>
        /// Add a bidirectional edge between two points
        /// </summary>
        public void AddBidirectionalEdge(Vector3I pos1, Vector3I pos2, float cost)
        {
            AddEdge(pos1, pos2, cost);
            AddEdge(pos2, pos1, cost);
        }

        /// <summary>
        /// Get all neighbors of a position
        /// </summary>
        public List<Edge> GetNeighbors(Vector3I position)
        {
            if (adjacencyList.ContainsKey(position))
            {
                return adjacencyList[position];
            }

            return new List<Edge>(); // Return empty list if no connections
        }

        /// <summary>
        /// Check if there's a direct connection between two points
        /// </summary>
        public bool HasEdge(Vector3I from, Vector3I to)
        {
            if (!adjacencyList.ContainsKey(from))
                return false;

            foreach (var edge in adjacencyList[from])
            {
                if (edge.Target.Equals(to))
                    return true;
            }

            return false;
        }

        /// <summary>
        /// Get the cost of a direct edge, or infinity if no connection
        /// </summary>
        public float GetEdgeCost(Vector3I from, Vector3I to)
        {
            if (!adjacencyList.ContainsKey(from))
                return float.PositiveInfinity;

            foreach (var edge in adjacencyList[from])
            {
                if (edge.Target.Equals(to))
                    return edge.Weight;
            }

            return float.PositiveInfinity;
        }

        /// <summary>
        /// Remove an edge from the graph
        /// </summary>
        public void RemoveEdge(Vector3I from, Vector3I to)
        {
            if (!adjacencyList.ContainsKey(from))
                return;

            adjacencyList[from].RemoveAll(e => e.Target.Equals(to));

            // Clean up empty lists
            if (adjacencyList[from].Count == 0)
            {
                adjacencyList.Remove(from);
            }
        }

        /// <summary>
        /// Get all nodes in the graph
        /// </summary>
        public IEnumerable<Vector3I> GetAllNodes()
        {
            return adjacencyList.Keys;
        }

        /// <summary>
        /// Helper: Create a standard 6-directional grid in a region
        /// </summary>
        public void CreateStandardGrid(Vector3I min, Vector3I max, float moveCost = 1.0f)
        {
            // Six directions: +X, -X, +Y, -Y, +Z, -Z
            Vector3I[] directions = new Vector3I[]
            {
            new Vector3I(1, 0, 0),   // Right
            new Vector3I(-1, 0, 0),  // Left
            new Vector3I(0, 1, 0),   // Up
            new Vector3I(0, -1, 0),  // Down
            new Vector3I(0, 0, 1),   // Forward
            new Vector3I(0, 0, -1)   // Back
            };

            for (int x = min.X; x <= max.X; x++)
            {
                for (int y = min.Y; y <= max.Y; y++)
                {
                    for (int z = min.Z; z <= max.Z; z++)
                    {
                        var current = new Vector3I(x, y, z);

                        // Connect to all 6 adjacent cells
                        foreach (var dir in directions)
                        {
                            var neighbor = new Vector3I(
                                current.X + dir.X,
                                current.Y + dir.Y,
                                current.Z + dir.Z
                            );

                            // Check if neighbor is within bounds
                            if (neighbor.X >= min.X && neighbor.X <= max.X &&
                                neighbor.Y >= min.Y && neighbor.Y <= max.Y &&
                                neighbor.Z >= min.Z && neighbor.Z <= max.Z)
                            {
                                AddBidirectionalEdge(current, neighbor, moveCost);
                            }
                        }
                    }
                }
            }
        }
    }
}
