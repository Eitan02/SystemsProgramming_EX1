// eitan.derdiger@gmail.com

#include <iostream>
#include "Graph.h"
#include "Algorithms.h"
#include "DataStructures.h"

using namespace graph;

int main() {
    try {
        // Create a graph with 5 vertices
        Graph g(5);

        // Add some undirected weighted edges
        g.addEdge(0, 1, 2);
        g.addEdge(0, 2, 3);
        g.addEdge(1, 3, 4);
        g.addEdge(2, 3, 1);
        g.addEdge(3, 4, 7);

        std::cout << "Original Graph:\n";
        g.print_graph();

        // Run BFS starting from vertex 0 and print the resulting tree
        Graph* bfsTree = Algorithms::bfs(g, 0);
        std::cout << "\nBFS Tree from vertex 0:\n";
        bfsTree->print_graph();

        // Run DFS starting from vertex 0 and print the resulting tree
        Graph* dfsTree = Algorithms::dfs(g, 0);
        std::cout << "\nDFS Tree from vertex 0:\n";
        dfsTree->print_graph();

        // Run Dijkstra's algorithm from vertex 0
        Graph* dijkstraTree = Algorithms::dijkstra(g, 0);
        std::cout << "\nDijkstra Shortest Path Tree from vertex 0:\n";
        dijkstraTree->print_graph();

        // Run Prim's algorithm starting from vertex 0
        Graph* primMST = Algorithms::prim(g, 0);
        std::cout << "\nPrim MST (start from 0):\n";
        primMST->print_graph();

        // Run Kruskal's algorithm
        Graph* kruskalMST = Algorithms::kruskal(g);
        std::cout << "\nKruskal MST:\n";
        kruskalMST->print_graph();

        // Clean up dynamic memory
        delete bfsTree;
        delete dfsTree;
        delete dijkstraTree;
        delete primMST;
        delete kruskalMST;

        // Remove an edge and print the updated graph
        std::cout << "\nRemoving edge (2,3):\n";
        g.removeEdge(2, 3);
        g.print_graph();

    } catch (std::exception& e) {
        // Catch and print any runtime errors
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
