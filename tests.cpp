// eitan.derdiger@gmail.com

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"               // Framework for writing unit tests
#include "Graph.h"                 // The Graph class definition
#include "Algorithms.h"           // BFS, DFS, Dijkstra, Prim, Kruskal
#include "DataStructures.h"       // Queue, PriorityQueue, DisjointSet
#include <stdexcept>              // For exception handling

using namespace graph;            // So we can avoid writing graph:: everywhere

// === GRAPH TESTS ===
// Test adding/removing edges and handling of invalid operations
TEST_CASE("Graph - add, remove and invalid edges") {
    Graph g(4); // create a graph with 4 vertices

    g.addEdge(0, 1, 10);    // valid edge
    g.addEdge(1, 2, 5);     // valid edge
    g.addEdge(2, 3, 3);     // valid edge

    // Try adding edges with invalid indices
    CHECK_THROWS_AS(g.addEdge(-1, 2), std::runtime_error);  // negative index
    CHECK_THROWS_AS(g.addEdge(0, 5), std::runtime_error);   // index out of bounds

    // Remove a valid edge and then try removing it again (should throw)
    CHECK_NOTHROW(g.removeEdge(1, 2));                      // valid removal
    CHECK_THROWS_AS(g.removeEdge(1, 2), std::runtime_error); // already removed
}

// === ALGORITHM TESTS ===
// BFS tree should connect all reachable nodes in level order
TEST_CASE("BFS - structure and invalid input") {
    Graph g(5);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 4);

    Graph* bfsTree = Algorithms::bfs(g, 0); // BFS starting from vertex 0

    // Check if tree edges exist (these will exist for sure in a path graph)
    CHECK_NOTHROW(bfsTree->removeEdge(0,1));
    CHECK_NOTHROW(bfsTree->removeEdge(1,2));
    CHECK_NOTHROW(bfsTree->removeEdge(2,3));
    CHECK_NOTHROW(bfsTree->removeEdge(3,4));

    // Check invalid starting vertex
    CHECK_THROWS_AS(Algorithms::bfs(g, 5), std::runtime_error);

    delete bfsTree;
}

// DFS should traverse deeply before moving to next branch
TEST_CASE("DFS - correct DFS tree structure") {
    Graph g(5);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 4);

    Graph* dfsTree = Algorithms::dfs(g, 0); // DFS starting from vertex 0

    // These edges will definitely be part of the DFS tree in this linear graph
    CHECK_NOTHROW(dfsTree->removeEdge(0,1));
    CHECK_NOTHROW(dfsTree->removeEdge(1,2));
    CHECK_NOTHROW(dfsTree->removeEdge(2,3));
    CHECK_NOTHROW(dfsTree->removeEdge(3,4));

    // Try invalid vertex
    CHECK_THROWS_AS(Algorithms::dfs(g, -1), std::runtime_error);

    delete dfsTree;
}

// Dijkstra should produce correct shortest paths (no negative weights!)
TEST_CASE("Dijkstra - valid and invalid behavior") {
    Graph g(4);
    g.addEdge(0, 1, 2);
    g.addEdge(1, 2, 1);
    g.addEdge(2, 3, 3);

    Graph* dijkstraTree = Algorithms::dijkstra(g, 0);

    // Check that valid tree edges exist
    CHECK_NOTHROW(dijkstraTree->removeEdge(0,1));
    CHECK_NOTHROW(dijkstraTree->removeEdge(1,2));
    CHECK_NOTHROW(dijkstraTree->removeEdge(2,3));

    delete dijkstraTree;

    // Test with negative weight (should throw)
    Graph gNeg(2);
    gNeg.addEdge(0, 1, -5);
    CHECK_THROWS_AS(Algorithms::dijkstra(gNeg, 0), std::runtime_error);

    // Out-of-bounds start vertex
    CHECK_THROWS_AS(Algorithms::dijkstra(g, 5), std::runtime_error);
}

// Prim should produce MST starting from a given vertex
TEST_CASE("Prim - spanning tree and input check") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 5);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 4);
    g.addEdge(2, 3, 1);

    Graph* primTree = Algorithms::prim(g, 0);

    // Check the number of vertices (should match original)
    CHECK(primTree->getNumVertices() == 4);

    // Invalid vertex
    CHECK_THROWS_AS(Algorithms::prim(g, 6), std::runtime_error);

    delete primTree;
}

// Kruskal MST works globally (independent of start vertex)
TEST_CASE("Kruskal - spanning tree structure") {
    Graph g(4);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 2, 6);
    g.addEdge(0, 3, 5);
    g.addEdge(1, 3, 15);
    g.addEdge(2, 3, 4);

    Graph* kruskalTree = Algorithms::kruskal(g);

    CHECK(kruskalTree->getNumVertices() == 4);

    delete kruskalTree;
}

// === QUEUE TESTS ===
// Test basic queue functionality: enqueue, dequeue, full/empty behavior
TEST_CASE("Queue - behavior and overflow/underflow") {
    Queue q(3); // capacity = 3

    CHECK(q.isEmpty());

    q.enqueue(1);
    q.enqueue(2);
    q.enqueue(3);

    CHECK(q.isFull());

    CHECK_THROWS_AS(q.enqueue(4), std::runtime_error);  // Should throw: full

    CHECK(q.dequeue() == 1);
    CHECK(q.dequeue() == 2);
    CHECK(q.dequeue() == 3);

    CHECK(q.isEmpty());
    CHECK_THROWS_AS(q.dequeue(), std::runtime_error);   // Should throw: empty
}

// === PRIORITY QUEUE TESTS ===
// Ensure min element is always returned
TEST_CASE("PriorityQueue - basic priority behavior") {
    PriorityQueue pq(3);

    pq.push(1, 10);
    pq.push(2, 5);
    pq.push(3, 7);

    CHECK_THROWS_AS(pq.push(4, 1), std::runtime_error); // Should throw: full

    // Min distances come out first
    CHECK(pq.pop().vertex == 2); // distance = 5
    CHECK(pq.pop().vertex == 3); // distance = 7
    CHECK(pq.pop().vertex == 1); // distance = 10

    CHECK(pq.isEmpty());
    CHECK_THROWS_AS(pq.pop(), std::runtime_error); // Should throw: empty
}

// === DISJOINT SET TESTS ===
// Test union-find correctness
TEST_CASE("DisjointSet - union and find operations") {
    DisjointSet ds(5); // sets: {0}, {1}, {2}, {3}, {4}

    CHECK(ds.findSet(0) == 0);

    ds.unionSet(0, 1); // sets: {0,1}
    ds.unionSet(1, 2); // sets: {0,1,2}

    // All three should now belong to same set
    CHECK(ds.findSet(0) == ds.findSet(2));

    // 3 and 0 are still in separate sets
    CHECK(ds.findSet(3) != ds.findSet(0));
}

// === MAIN FUNCTION FOR DOCTEST ===
// Required to run all the above tests
int main(int argc, char** argv) {
    doctest::Context context;
    context.applyCommandLine(argc, argv);   // Pass arguments from command line if needed
    int res = context.run();                // Run all test cases

    if (context.shouldExit()) return res;   // Exit early if needed (e.g., --exit-after-tests)
    return res;
}
