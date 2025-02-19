
tests.o: maze.hpp
	g++ -std=c++23 -o tests.o main.cpp maze.hpp -g -lgtest

clean: 
	rm *.o