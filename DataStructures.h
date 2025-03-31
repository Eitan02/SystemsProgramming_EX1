// eitan.derdiger@gmail.com

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

namespace graph {

// Simple circular array-based Queue
class Queue {
private:
    int* data;          // Array to store elements
    int capacity;       // Maximum capacity of the queue
    int frontIndex;     // Index of the front element
    int rearIndex;      // Index of the rear element
    int size;           // Current size of the queue

public:
    Queue(int cap);     // Constructor
    ~Queue();           // Destructor

    void enqueue(int val);      // Add element to the queue
    int dequeue();              // Remove and return element from the front
    bool isEmpty() const;       // Check if the queue is empty
    bool isFull() const;        // Check if the queue is full
};

// Struct representing an item in the Priority Queue
struct PQItem {
    int vertex;     // Vertex index
    int distance;   // Priority value (e.g., distance in Dijkstra)
};

// Simple unsorted array-based Priority Queue
class PriorityQueue {
private:
    PQItem* data;   // Array of PQItem structs
    int capacity;   // Maximum number of items
    int count;      // Current number of items

public:
    PriorityQueue(int cap);     // Constructor
    ~PriorityQueue();           // Destructor

    void push(int v, int dist); // Add a new item to the queue
    PQItem pop();               // Remove and return item with lowest distance
    bool isEmpty() const;       // Check if the queue is empty
};

// Disjoint Set (Union-Find) structure for Kruskal's algorithm
class DisjointSet {
private:
    int* parent;    // Array for parent pointers
    int* rank;      // Array for rank (used to optimize union)
    int size;       // Number of elements in the set

public:
    DisjointSet(int n);         // Constructor
    ~DisjointSet();             // Destructor

    int findSet(int x);         // Find representative of set containing x
    void unionSet(int x, int y); // Union of sets containing x and y
};

}

#endif
