
#ifndef __RRT_BASE

#define __RRT_BASE
#include "environment.hpp"
#include "point_set.hpp"
#include "collision_engine.hpp"

class rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   virtual void generate_next(int num) = 0;
};

#endif
