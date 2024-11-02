
#ifndef __WEIGHTED_EDGE

#define __WEIGHTED_EDGE
#include "edge.hpp"

class weighted_edge : public edge {
public:
   double cost;

   weighted_edge() : edge(), cost(0) {}
   weighted_edge(vertex from, vertex to)
      : edge(from,to), cost(0) {}
   weighted_edge(vertex from, vertex to, edge * parent)
      : edge(from,to,parent), cost(0) {}
};

#endif
