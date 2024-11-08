using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class TerrainPathfindingVisualizer : MonoBehaviour
{
    public Terrain terrain;
    public Transform startPoint, endPoint;
    public int gridSizeX = 50;
    public int gridSizeY = 50;
    public float nodeSize = 1f;
    public Color dijkstraColor = Color.blue;
    public Color aStarColor = Color.red;

    private Node[,] grid;
    private LineRenderer dijkstraPathRenderer;
    private LineRenderer aStarPathRenderer;

    public DistanceType currentDistanceType = DistanceType.Euclidean;

    void Start()
    {
        CreateGrid();
        InitializePathRenderers();
        CompareAlgorithms();
    }

    void InitializePathRenderers()
    {
        dijkstraPathRenderer = CreatePathRenderer(dijkstraColor);
        aStarPathRenderer = CreatePathRenderer(aStarColor);
    }

    LineRenderer CreatePathRenderer(Color color)
    {
        GameObject lineObj = new GameObject("PathRenderer");
        LineRenderer lineRenderer = lineObj.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 0;
        return lineRenderer;
    }

    bool IsWalkable(Vector3 position)
    { 
        float detectionRadius = nodeSize * 0.5f;
 
        Collider[] colliders = Physics.OverlapSphere(position, detectionRadius);
        foreach (Collider collider in colliders)
        {
            if (collider.CompareTag("Unwalkable"))
            {
                return false; 
            }
        }
        return true;
    }

    void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeY];
        Vector3 bottomLeft = terrain.transform.position;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                float worldX = bottomLeft.x + x * nodeSize;
                float worldZ = bottomLeft.z + y * nodeSize;
                float height = terrain.SampleHeight(new Vector3(worldX, 0, worldZ)) + terrain.transform.position.y;
                Vector3 worldPosition = new Vector3(worldX, height, worldZ);
 
                bool walkable = IsWalkable(worldPosition);
                grid[x, y] = new Node(walkable, worldPosition, x, y);

                // Visualisasi Node
                GameObject gridIndicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                gridIndicator.transform.position = worldPosition;
                gridIndicator.transform.localScale = Vector3.one * nodeSize * 0.5f;

                Renderer renderer = gridIndicator.GetComponent<Renderer>();
                if (walkable)
                {
                    renderer.material.color = Color.green;
                }
                else
                {
                    renderer.material.color = Color.red;
                    gridIndicator.tag = "Unwalkable"; 
                }
 
                TextMeshPro costText = new GameObject("CostText").AddComponent<TextMeshPro>();
                costText.text = $"g: {grid[x, y].gCost}\nh: {grid[x, y].hCost}\nf: {grid[x, y].fCost}";
                costText.fontSize = 5;
                costText.color = Color.black;
                costText.alignment = TextAlignmentOptions.Center;
                costText.transform.position = worldPosition + Vector3.up * 3f;

                grid[x, y].costText = costText;
                Destroy(gridIndicator.GetComponent<Collider>());
            }
        }
    }



    Node GetNodeFromWorldPosition(Vector3 position)
    {
        float percentX = Mathf.Clamp01((position.x - terrain.transform.position.x) / (gridSizeX * nodeSize));
        float percentY = Mathf.Clamp01((position.z - terrain.transform.position.z) / (gridSizeY * nodeSize));
        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    void CompareAlgorithms()
    {
        Node startNode = GetNodeFromWorldPosition(startPoint.position);
        Node endNode = GetNodeFromWorldPosition(endPoint.position);

        List<Vector3> pathDijkstra = CalculateDijkstraPath(startNode, endNode);
        DrawPath(dijkstraPathRenderer, pathDijkstra, dijkstraColor);

        List<Vector3> pathAStar = CalculateAStarPath(startNode, endNode);
        DrawPath(aStarPathRenderer, pathAStar, aStarColor);
        UpdateCostText();
    }

    void DrawPath(LineRenderer lineRenderer, List<Vector3> path, Color color)
    {
        if (path == null || path.Count == 0)
            return;

        lineRenderer.positionCount = path.Count;
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = 0.2f;  // Sesuaikan ketebalan garis
        lineRenderer.endWidth = 0.2f;
        lineRenderer.SetPositions(path.ToArray());

        // Tambahkan garis yang menghubungkan setiap node ke node berikutnya
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], color, 5f); // Menggunakan Debug.DrawLine untuk melihat garis di Scene View
            DrawConnectingLine(path[i], path[i + 1], color);
        }
    }

    void DrawConnectingLine(Vector3 start, Vector3 end, Color color)
    {
        GameObject lineObj = new GameObject("ConnectingLine");
        LineRenderer lineRenderer = lineObj.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);
    }

    void UpdateCostText()
    {
        foreach (Node node in grid)
        {
            TextMeshPro costText = node.costText;
            if (costText != null)
                costText.text = $"g: {node.gCost}\nh: {node.hCost}\nf: {node.fCost}";
        }
    }


    List<Vector3> CalculateDijkstraPath(Node startNode, Node endNode)
    {
        var openSet = new List<Node> { startNode };
        var closedSet = new HashSet<Node>();
        startNode.gCost = 0;

        while (openSet.Count > 0)
        {
            Node currentNode = GetNodeWithLowestCost(openSet);
            if (currentNode == endNode) return RetracePath(startNode, endNode);

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (closedSet.Contains(neighbor) || !neighbor.walkable) continue;
                int newCost = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newCost < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCost;
                    neighbor.parent = currentNode;
                    if (!openSet.Contains(neighbor)) openSet.Add(neighbor);
                }
            }
        }
        return new List<Vector3>();
    }

    List<Vector3> CalculateAStarPath(Node startNode, Node endNode)
    {
        var openSet = new List<Node> { startNode };
        var closedSet = new HashSet<Node>();
        startNode.gCost = 0;
        startNode.hCost = GetDistance(startNode, endNode);

        while (openSet.Count > 0)
        {
            Node currentNode = GetNodeWithLowestCost(openSet);
            if (currentNode == endNode) return RetracePath(startNode, endNode);

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (closedSet.Contains(neighbor) || !neighbor.walkable) continue;
                int newCost = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newCost < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCost;
                    neighbor.hCost = GetDistance(neighbor, endNode);
                    neighbor.parent = currentNode;
                    if (!openSet.Contains(neighbor)) openSet.Add(neighbor);
                }
            }
        }
        return new List<Vector3>();
    }

    int GetDistance(Node a, Node b) => Mathf.Abs(a.gridX - b.gridX) + Mathf.Abs(a.gridY - b.gridY);

    Node GetNodeWithLowestCost(List<Node> nodes)
    {
        Node bestNode = nodes[0];
        foreach (Node node in nodes)
            if (node.fCost < bestNode.fCost) bestNode = node;
        return bestNode;
    }

    List<Vector3> RetracePath(Node start, Node end)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = end;
        while (currentNode != start)
        {
            path.Add(currentNode.worldPosition);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }

    List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0) continue;
                int checkX = node.gridX + x;
                int checkY = node.gridY + y;
                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                    neighbors.Add(grid[checkX, checkY]);
            }
        }
        return neighbors;
    }

    public enum DistanceType { Euclidean, Manhattan }

    public class Node
    {
        public bool walkable;
        public Vector3 worldPosition;
        public int gridX, gridY;
        public int gCost, hCost;
        public Node parent;
        public TextMeshPro costText;

        public Node(bool walkable, Vector3 worldPosition, int gridX, int gridY)
        {
            this.walkable = walkable;
            this.worldPosition = worldPosition;
            this.gridX = gridX;
            this.gridY = gridY;
        }

        public int fCost => gCost + hCost;
    }
}
