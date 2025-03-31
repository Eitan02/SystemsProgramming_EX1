// eitan.derdiger@gmail.com

#include <iostream>              // For std::cout (used in print_graph)
#include <exception>            // For throwing std::runtime_error
#include "Graph.h"              // The graph class declaration

namespace graph {

// Constructor: create a graph with a given number of vertices
Graph::Graph(int vertices)
    : numVertices(vertices)
{
    if (vertices <= 0) {
        throw std::runtime_error("Number of vertices must be positive.");
    }

    // Allocate an array of pointers for adjacency lists (each entry is a linked list head)
    adjacencyList = new AdjNode*[numVertices];

    // Initialize each list to nullptr (empty)
    for (int i = 0; i < numVertices; i++) {
        adjacencyList[i] = nullptr;
    }
}

// Destructor: frees all memory allocated for adjacency lists
Graph::~Graph()
{
    // Loop through each list and delete its nodes
    for (int i = 0; i < numVertices; i++) {
        AdjNode* current = adjacencyList[i];
        while (current) {
            AdjNode* temp = current;
            current = current->next;
            delete temp; // delete each node
        }
    }
    delete[] adjacencyList; // delete the array of list heads
}

// Adds an undirected edge between src and dest
void Graph::addEdge(int src, int dest, int weight)
{
    // Check that vertex indices are valid
    if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
        throw std::runtime_error("addEdge: Invalid vertex index.");
    }

    // Disallow self-loops (edge from vertex to itself)
    if (src == dest) {
        throw std::runtime_error("addEdge: Self-loops are not allowed.");
    }

    // Check if edge already exists in src's list
    AdjNode* current = adjacencyList[src];
    while (current) {
        if (current->dest == dest) {
            throw std::runtime_error("addEdge: Edge already exists.");
        }
        current = current->next;
    }

    // Insert new node at the beginning of src's adjacency list
    AdjNode* newNode = new AdjNode(dest, weight, adjacencyList[src]);
    adjacencyList[src] = newNode;

    // Since the graph is undirected, add edge in reverse direction as well
    AdjNode* newNode2 = new AdjNode(src, weight, adjacencyList[dest]);
    adjacencyList[dest] = newNode2;
}

// Removes an edge (undirected) between src and dest
void Graph::removeEdge(int src, int dest)
{
    // Validate vertex indices
    if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
        throw std::runtime_error("removeEdge: Invalid vertex index.");
    }

    // Lambda function to remove an edge from a specific adjacency list
    auto removeFromList = [&](int start, int finish) {
        AdjNode* current = adjacencyList[start];
        AdjNode* prev = nullptr;

        // Traverse the list to find the node
        while (current) {
            if (current->dest == finish) {
                if (prev == nullptr) {
                    adjacencyList[start] = current->next; // remove head
                } else {
                    prev->next = current->next;           // remove from middle/end
                }
                delete current;
                return true;
            }
            prev = current;
            current = current->next;
        }
        return false; // not found
    };

    // Remove both directions
    bool removedSrc = removeFromList(src, dest);
    bool removedDest = removeFromList(dest, src);

    // If either direction was missing, throw
    if (!removedSrc || !removedDest) {
        throw std::runtime_error("removeEdge: Edge does not exist.");
    }
}

// Prints the graph’s adjacency list to console
void Graph::print_graph() const
{
    std::cout << "Graph Adjacency List:\n";
    for (int i = 0; i < numVertices; i++) {
        std::cout << "[" << i << "] -> ";

        AdjNode* current = adjacencyList[i];
        while (current) {
            std::cout << "(" << current->dest << ", w=" << current->weight << ") ";
            current = current->next;
        }

        std::cout << "\n";
    }
}

}
