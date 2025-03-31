// eitan.derdiger@gmail.com

#include <stdexcept>
#include "DataStructures.h"

namespace graph {

// ===== QUEUE =====

// Constructor: Initialize queue with a fixed capacity
Queue::Queue(int cap)
    : capacity(cap), frontIndex(0), rearIndex(-1), size(0)
{
    if (cap <= 0) {
        throw std::runtime_error("Queue capacity must be positive.");
    }
    data = new int[cap];  // Allocate memory for the queue
}

// Destructor: Free allocated memory
Queue::~Queue()
{
    delete[] data;
}

// Enqueue: Add an element to the rear of the queue
void Queue::enqueue(int val)
{
    if (isFull()) {
        throw std::runtime_error("Queue is full.");
    }
    // Circular insert
    rearIndex = (rearIndex + 1) % capacity;
    data[rearIndex] = val;
    size++;
}

// Dequeue: Remove and return the front element
int Queue::dequeue()
{
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty.");
    }
    int val = data[frontIndex];
    frontIndex = (frontIndex + 1) % capacity;
    size--;
    return val;
}

bool Queue::isEmpty() const
{
    return (size == 0);
}

bool Queue::isFull() const
{
    return (size == capacity);
}

// ===== PRIORITY QUEUE =====

// Constructor: Initialize with fixed capacity
PriorityQueue::PriorityQueue(int cap)
    : capacity(cap), count(0)
{
    if (cap <= 0) {
        throw std::runtime_error("PriorityQueue capacity must be positive.");
    }
    data = new PQItem[cap];
}

// Destructor: Free memory
PriorityQueue::~PriorityQueue()
{
    delete[] data;
}

// Push: Add a new element to the queue
void PriorityQueue::push(int v, int dist)
{
    if (count == capacity) {
        throw std::runtime_error("PriorityQueue is full.");
    }
    data[count].vertex = v;
    data[count].distance = dist;
    count++;
}

// Pop: Remove and return the element with minimum distance
PQItem PriorityQueue::pop()
{
    if (isEmpty()) {
        throw std::runtime_error("PriorityQueue is empty.");
    }

    // Find the index with the minimum distance
    int minIndex = 0;
    for (int i = 1; i < count; i++) {
        if (data[i].distance < data[minIndex].distance) {
            minIndex = i;
        }
    }

    PQItem result = data[minIndex];

    // Replace removed element with the last one to keep array compact
    data[minIndex] = data[count - 1];
    count--;

    return result;
}

bool PriorityQueue::isEmpty() const
{
    return (count == 0);
}

// ===== DISJOINT SET (UNION FIND) =====

// Constructor: Initialize each element as its own parent
DisjointSet::DisjointSet(int n)
    : size(n)
{
    parent = new int[n];
    rank = new int[n];
    for (int i = 0; i < n; i++) {
        parent[i] = i;   // Each node is its own root initially
        rank[i] = 0;     // Initial rank is 0
    }
}

// Destructor: Clean up arrays
DisjointSet::~DisjointSet()
{
    delete[] parent;
    delete[] rank;
}

// Find: Return root of the set and compress the path
int DisjointSet::findSet(int x)
{
    if (parent[x] != x) {
        parent[x] = findSet(parent[x]); // Path compression
    }
    return parent[x];
}

// Union: Merge sets containing x and y using union by rank
void DisjointSet::unionSet(int x, int y)
{
    int rx = findSet(x);
    int ry = findSet(y);

    if (rx == ry) return; // Already in the same set

    // Merge smaller tree into larger
    if (rank[rx] < rank[ry]) {
        parent[rx] = ry;
    } else if (rank[rx] > rank[ry]) {
        parent[ry] = rx;
    } else {
        parent[ry] = rx;
        rank[rx]++;
    }
}

}
