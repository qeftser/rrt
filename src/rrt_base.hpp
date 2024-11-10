
#ifndef __RRT_BASE

#define __RRT_BASE
#include "vertex.hpp"

class rrt_base {
public:

   virtual void generate_next(int num) = 0;
   virtual void restart(const vertex pos) = 0;
   virtual void notify_obstacle(const vertex pos) { };
};

#endif
