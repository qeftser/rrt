
#ifndef __RRT_X_EDGE

#define __RRT_X_EDGE
#include "edge.hpp"
#include <set>

class rrt_x_edge : public edge {
public:
   
   enum flag { inactive=1, colliding=2 };
   int flags;

   double g;
   double lmc;

   std::set<rrt_x_edge *> n0_in, n0_out;
   std::set<rrt_x_edge *> nr_in, nr_out;

   rrt_x_edge() : edge() {}
   rrt_x_edge(vertex from, vertex to)
      : edge(from,to) {}
   rrt_x_edge(vertex from, vertex to, edge * parent)
      : edge(from,to,parent) {}

};

#endif
