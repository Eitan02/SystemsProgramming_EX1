// eitan.derdiger@gmail.com

#ifndef GRAPH_H
#define GRAPH_H

namespace graph {

// A single node in the adjacency list
struct AdjNode {
    int dest;               // Destination vertex (neighbor)
    int weight;             // Weight of the edge
    AdjNode* next;          // Pointer to next neighbor in the list

    // Constructor to initialize the node
    AdjNode(int d, int w, AdjNode* n = nullptr)
        : dest(d), weight(w), next(n) {}
};

// Graph represented as adjacency lists (undirected)
class Graph {
private:
    int numVertices;            // Total number of vertices
    AdjNode** adjacencyList;    // Array of linked lists (one for each vertex)

public:
    // Constructor: initializes an empty graph with given number of vertices
    Graph(int vertices);

    // Destructor: frees all allocated memory (linked lists)
    ~Graph();

    // Adds an undirected edge between src and dest with optional weight
    void addEdge(int src, int dest, int weight = 1);

    // Removes the edge between src and dest (in both directions)
    void removeEdge(int src, int dest);

    // Prints the adjacency list of the graph (for debugging/demo)
    void print_graph() const;

    // Returns number of vertices
    int getNumVertices() const { return numVertices; }

    // Returns the internal adjacency list pointer (for algorithms)
    AdjNode** getAdjList() const { return adjacencyList; }
};

}

#endif
