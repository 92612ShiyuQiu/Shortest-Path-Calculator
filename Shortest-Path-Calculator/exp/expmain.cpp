// expmain.cpp
//
// ICS 46 Spring 2018
// Project #5: Rock and Roll Stops the Traffic
//
// Do whatever you'd like here.  This is intended to allow you to experiment
// with your code, outside of the context of the broader program or Google
// Test.

#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include "Digraph.hpp"

int main()
{
	Digraph<int, double> d1;
    d1.addVertex(1, 10);
    d1.addVertex(2, 20);
    d1.addVertex(3, 30);
    d1.addVertex(4, 40);

    d1.addEdge(1, 2, 5.0);
    d1.addEdge(2, 3, 6.0);
    d1.addEdge(3, 4, 7.0);
    d1.addEdge(1, 4, 1.0);

    std::cout<<d1.isStronglyConnected()<<std::endl;
    
    
    return 0;
}

