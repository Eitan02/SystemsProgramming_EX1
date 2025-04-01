# Graph Assignment- README

## Overview
This project implements an undirected weighted graph using an Adjacency List in C++ without using the C++ standard library containers.  
It includes:
- A `Graph` class managing vertices and edges (undirected).
- An `Algorithms` class providing `BFS`, `DFS`, `Dijkstra`, `Prim`, and `Kruskal`.
- Basic data structures (`Queue`, `PriorityQueue`, `DisjointSet`) implemented with raw arrays and pointers.
- A `main.cpp` demonstrating usage.
- `tests.cpp` containing unit tests using doctest.
- A `Makefile` to compile, test, and run valgrind checks.

---

## File Structure

.
├── main.cpp              # Demonstration of graph and algorithms

├── Graph.h / Graph.cpp   # Graph implementation
├── Algorithms.h/.cpp     # BFS, DFS, Dijkstra, Prim, Kruskal
├── DataStructures.h/.cpp # Queue, PriorityQueue, DisjointSet
├── tests.cpp             # Unit tests with doctest
├── Makefile              # Compilation, testing, valgrind


---
## How to Build

1. Make sure you have clang++, make, and valgrind installed.
2. Run the following to build everything: `make Main`
3. Then run: `./Main`
This will execute the demonstration in `main.cpp` showing graph creation and algorithm usage.

---

## How to Test

- Run all tests: `make test`

- Run with valgrind to check for memory leaks: `make valgrind`

- Clean up compiled files: `make clean`


---


## Author
- Eitan Derdiger - 328363700 - eitan.derdiger@gmail.com

---

## Notes
- All inputs are validated, and errors throw `std::runtime_error` exceptions.
- Dynamic memory is correctly freed (checked with valgrind).
- No use of STL as required by assignment instructions.
- Code is clean, modular, and documented.
