# eitan.derdiger@gmail.com

CXX = clang++
CXXFLAGS = -std=c++11 -Wall -Wextra -I.

OBJS = Graph.o Algorithms.o DataStructures.o
MAIN_OBJ = main.o
TESTS_OBJ = tests.o

Main: $(OBJS) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) -o Main $(OBJS) $(MAIN_OBJ)

Tests: $(OBJS) $(TESTS_OBJ)
	$(CXX) $(CXXFLAGS) -o Tests $(OBJS) $(TESTS_OBJ)

Graph.o: Graph.cpp Graph.h
	$(CXX) $(CXXFLAGS) -c Graph.cpp

Algorithms.o: Algorithms.cpp Algorithms.h Graph.h DataStructures.h
	$(CXX) $(CXXFLAGS) -c Algorithms.cpp

DataStructures.o: DataStructures.cpp DataStructures.h
	$(CXX) $(CXXFLAGS) -c DataStructures.cpp

main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp

tests.o: tests.cpp
	$(CXX) $(CXXFLAGS) -c tests.cpp

test: Tests
	./Tests --success --reporters=console

valgrind: Main
	valgrind --leak-check=full ./Main

clean:
	rm -f *.o Main Tests
