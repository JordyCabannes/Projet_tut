INC = -I ../algorithms/DataStructures -I ../lib/core -I ../algorithms/RegLC -I ../algorithms/utils -I ../algorithms
OPT = -W -Wall -g -O0 -std=c++11 -fPIC

all: main lib.so

lib.so: GraphFactory.o graph_wrapper.o Landmark.o duration.o reglc_graph.o
	g++ $(opt) $^ -shared -Wl,-soname,libmumoro.so -o libmumoro.so -lboost_serialization

main: main.o lib.so
	g++ $(OPT) main.o -Wl,-rpath=$(shell pwd)/ -L $(shell pwd)/ -lmumoro -lboost_serialization -o $@

duration.o:
	g++ $(OPT) -c ../lib/core/duration.cpp -o duration.o $(INC)

reglc_graph.o:
	g++ $(OPT) -c ../algorithms/RegLC/reglc_graph.cpp -o reglc_graph.o $(INC)

Landmark.o:
	g++ $(OPT) -c ../algorithms/RegLC/Landmark.cpp -o Landmark.o $(INC)

GraphFactory.o:
	g++ $(OPT) -c ../algorithms/DataStructures/GraphFactory.cpp -o GraphFactory.o $(INC)

graph_wrapper.o:
	g++ $(OPT) -c ../lib/core/graph_wrapper.cpp -o graph_wrapper.o $(INC)

main.o: main.cpp main.hpp
	g++ $(OPT) -c main.cpp -o main.o $(INC)

clean-all: clean mrproper

clean:
	@rm -f *.o
	@rm -f *.so
	@rm -f main

mrproper:
	@rm -f *~