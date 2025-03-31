// eitan.derdiger@gmail.com

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.h"

namespace graph {

// This class provides static methods for common graph algorithms
class Algorithms {
public:
    // Returns a BFS tree rooted at startVertex
    static Graph* bfs(const Graph& g, int startVertex);

    // Returns a DFS tree (or forest) starting at startVertex
    static Graph* dfs(const Graph& g, int startVertex);

    // Returns the shortest path tree (SPT) from startVertex using Dijkstra's algorithm
    static Graph* dijkstra(const Graph& g, int startVertex);

    // Returns a minimum spanning tree using Prim's algorithm starting from startVertex
    static Graph* prim(const Graph& g, int startVertex);

    // Returns a minimum spanning tree using Kruskal's algorithm
    static Graph* kruskal(const Graph& g);

private:
    // Helper function for DFS traversal (recursive)
    static void dfsVisit(const Graph& g, int vertex, bool* visited, Graph* dfsTree);
};

}

#endif
