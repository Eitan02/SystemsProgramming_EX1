// eitan.derdiger@gmail.com

#include <limits>           // For INT_MAX (used in Dijkstra and Prim)
#include <stdexcept>        // For throwing exceptions on invalid input
#include "Algorithms.h"
#include "DataStructures.h"

namespace graph {

// Check if the graph contains any edge with a negative weight.
bool hasNegativeWeights(const Graph& g) {
    int V = g.getNumVertices();         // Number of vertices in the graph
    AdjNode** adjList = g.getAdjList(); // Adjacency list pointer

    // Go through each vertex
    for (int i = 0; i < V; i++) {
        AdjNode* curr = adjList[i];

        // Traverse each node in the adjacency list of vertex i
        while (curr) {
            if (curr->weight < 0) {
                return true; // Negative weight found
            }
            curr = curr->next;
        }
    }

    return false; // No negative weights found
}

Graph* Algorithms::bfs(const Graph& g, int startVertex)
{
    int V = g.getNumVertices();

    // Check for valid vertex index
    if (startVertex < 0 || startVertex >= V) {
        throw std::runtime_error("BFS: Invalid start vertex.");
    }

    Graph* bfsTree = new Graph(V); // Create a new graph to store the BFS tree
    bool* visited = new bool[V];   // Array to track visited nodes

    // Initialize all vertices as unvisited
    for (int i = 0; i < V; i++) {
        visited[i] = false;
    }

    Queue queue(V);               // Our custom-built queue
    visited[startVertex] = true;  // Mark the start vertex as visited
    queue.enqueue(startVertex);   // Start BFS from here

    AdjNode** adjList = g.getAdjList(); // Get original graph's adjacency list

    // Main BFS loop
    while (!queue.isEmpty()) {
        int u = queue.dequeue();        // Dequeue next vertex
        AdjNode* current = adjList[u];  // Traverse its neighbors

        while (current) {
            int v = current->dest;

            if (!visited[v]) {
                visited[v] = true;            // Mark neighbor as visited
                bfsTree->addEdge(u, v, 1);    // Add edge to BFS tree (weight = 1)
                queue.enqueue(v);             // Enqueue neighbor
            }

            current = current->next; // Go to next neighbor
        }
    }

    delete[] visited; // Clean up memory
    return bfsTree;   // Return the BFS tree
}


Graph* Algorithms::dfs(const Graph& g, int startVertex)
{
    int V = g.getNumVertices();

    if (startVertex < 0 || startVertex >= V) {
        throw std::runtime_error("DFS: Invalid start vertex.");
    }

    Graph* dfsTree = new Graph(V); // Result tree
    bool* visited = new bool[V];   // Track visited vertices

    // Initialize all to false
    for (int i = 0; i < V; i++) {
        visited[i] = false;
    }

    dfsVisit(g, startVertex, visited, dfsTree); // Recursive DFS

    delete[] visited;
    return dfsTree;
}

// Recursive helper function for DFS
void Algorithms::dfsVisit(const Graph& g, int vertex, bool* visited, Graph* dfsTree)
{
    visited[vertex] = true; // Mark current node as visited

    AdjNode** adjList = g.getAdjList();
    AdjNode* current = adjList[vertex]; // Get neighbors of this vertex

    while (current) {
        int v = current->dest;

        if (!visited[v]) {
            dfsTree->addEdge(vertex, v, 1); // Add edge to DFS tree
            dfsVisit(g, v, visited, dfsTree); // Recursive call
        }

        current = current->next;
    }
}


Graph* Algorithms::dijkstra(const Graph& g, int startVertex)
{
    int V = g.getNumVertices();

    if (startVertex < 0 || startVertex >= V) {
        throw std::runtime_error("Dijkstra: Invalid start vertex.");
    }

    // Dijkstra does not support negative weights!
    if (hasNegativeWeights(g)) {
        throw std::runtime_error("Dijkstra: Graph contains negative edge weights, which are not allowed.");
    }

    Graph* spt = new Graph(V); // Shortest Path Tree (result)
    int* distance = new int[V]; // Distance array
    int* parent = new int[V];   // To reconstruct path
    bool* visited = new bool[V];// Mark visited vertices

    for (int i = 0; i < V; i++) {
        distance[i] = std::numeric_limits<int>::max(); // Init distances to INF
        parent[i] = -1;
        visited[i] = false;
    }
    distance[startVertex] = 0;

    PriorityQueue pq(V * 10); // Our custom priority queue
    pq.push(startVertex, 0);

    AdjNode** adjList = g.getAdjList();

    while (!pq.isEmpty()) {
        PQItem top = pq.pop();
        int u = top.vertex;

        if (visited[u]) continue;
        visited[u] = true;

        AdjNode* current = adjList[u];
        while (current) {
            int v = current->dest;
            int w = current->weight;

            // Relaxation step
            if (!visited[v] && distance[u] + w < distance[v]) {
                distance[v] = distance[u] + w;
                parent[v] = u;
                pq.push(v, distance[v]);
            }

            current = current->next;
        }
    }

    // Build SPT from parent array
    for (int v = 0; v < V; v++) {
        if (parent[v] != -1) {
            int w = distance[v] - distance[parent[v]];
            spt->addEdge(parent[v], v, w);
        }
    }

    delete[] distance;
    delete[] parent;
    delete[] visited;
    return spt;
}


Graph* Algorithms::prim(const Graph& g, int startVertex)
{
    int V = g.getNumVertices();  // Get number of vertices in the graph

    // Check if the starting vertex is valid
    if (startVertex < 0 || startVertex >= V) {
        throw std::runtime_error("Prim: Invalid start vertex.");
    }

    Graph* mst = new Graph(V);   // This graph will store the MST result

    // Allocate memory for helper arrays
    bool* inMST = new bool[V];   // Keeps track of which vertices are already in the MST
    int* key = new int[V];       // Stores the minimum weight edge to connect each vertex
    int* parent = new int[V];    // Stores the parent of each vertex in the MST

    // Initialize arrays
    for (int i = 0; i < V; i++) {
        inMST[i] = false;                            // Initially no vertex is included in MST
        key[i] = std::numeric_limits<int>::max();    // Set key value to "infinity"
        parent[i] = -1;                              // No parent yet
    }

    key[startVertex] = 0;    // Start from this vertex (so its key is 0)

    PriorityQueue pq(V * 10); // Use our custom priority queue (not heap-based)
    pq.push(startVertex, 0);  // Push the start vertex with priority 0

    AdjNode** adjList = g.getAdjList();  // Get the adjacency list from the graph
    int countVisited = 0;                // Count how many vertices are included in MST

    // Main loop: continue until all vertices are in the MST or queue is empty
    while (!pq.isEmpty() && countVisited < V) {
        PQItem top = pq.pop();       // Get vertex with smallest key value
        int u = top.vertex;

        if (inMST[u]) continue;      // Skip if already included in MST

        inMST[u] = true;             // Mark this vertex as included in MST
        countVisited++;

        // If it has a parent, we can add the corresponding edge to the MST
        if (parent[u] != -1) {
            mst->addEdge(parent[u], u, key[u]);  // Add the edge from parent to u
        }

        AdjNode* current = adjList[u];  // Explore all neighbors of u
        while (current) {
            int v = current->dest;
            int w = current->weight;

            // If v is not in MST and weight(u,v) < current key[v], update key and parent
            if (!inMST[v] && w < key[v]) {
                key[v] = w;
                parent[v] = u;
                pq.push(v, key[v]);  // Push v with new lower key (priority)
            }

            current = current->next;  // Move to next neighbor
        }
    }

    // Clean up memory
    delete[] inMST;
    delete[] key;
    delete[] parent;

    return mst;  // Return the minimum spanning tree graph
}


Graph* Algorithms::kruskal(const Graph& g)
{
    int V = g.getNumVertices();           // Number of vertices
    int edgeCount = 0;                    // We'll count how many edges are in the graph
    AdjNode** adjList = g.getAdjList();   // Get adjacency list

    // First, count how many *unique* edges we have
    // Since the graph is undirected and each edge appears twice, we count only once
    for (int i = 0; i < V; i++) {
        AdjNode* current = adjList[i];
        while (current) {
            if (current->dest > i) {      // Only count if dest > src to avoid duplicates
                edgeCount++;
            }
            current = current->next;
        }
    }

    // Temporary structure to store all edges
    struct Edge {
        int u;  // source
        int v;  // destination
        int w;  // weight
    };

    Edge* edges = new Edge[edgeCount];  // Create an array to hold all edges
    int idx = 0;

    // Fill the array with all unique edges
    for (int i = 0; i < V; i++) {
        AdjNode* current = adjList[i];
        while (current) {
            if (current->dest > i) {  // Again, avoid duplicates
                edges[idx].u = i;
                edges[idx].v = current->dest;
                edges[idx].w = current->weight;
                idx++;
            }
            current = current->next;
        }
    }

    // Sort edges by weight using bubble sort (not efficient, but allowed)
    for (int i = 0; i < edgeCount - 1; i++) {
        for (int j = 0; j < edgeCount - i - 1; j++) {
            if (edges[j].w > edges[j+1].w) {
                Edge temp = edges[j];
                edges[j] = edges[j+1];
                edges[j+1] = temp;
            }
        }
    }

    Graph* mst = new Graph(V);      // The graph that will contain the MST
    DisjointSet ds(V);              // Union-Find structure to avoid cycles

    // Now go over the sorted edge list and build MST
    for (int i = 0; i < edgeCount; i++) {
        int u = edges[i].u;
        int v = edges[i].v;
        int w = edges[i].w;

        // If u and v are in different sets, adding this edge won't form a cycle
        if (ds.findSet(u) != ds.findSet(v)) {
            ds.unionSet(u, v);          // Union the sets
            mst->addEdge(u, v, w);      // Add the edge to MST
        }
    }

    delete[] edges;  // Clean up memory
    return mst;      // Return the MST graph
}

}
