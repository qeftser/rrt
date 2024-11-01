
#ifndef __COLLISION_ENGINE

#define __COLLISION_ENGINE
#include "environment.hpp"
#include "vertex.hpp"
#include "edge.hpp"

class collision_engine {
public:
   environment * env;

   collision_engine(environment * env) : env(env) {}

   virtual void recalibrate() = 0;
   virtual bool is_collision(edge *) = 0;
   virtual bool is_collision(const vertex &, const vertex &) = 0;
};

#endif
