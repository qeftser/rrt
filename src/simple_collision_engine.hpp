
#ifndef __SIMPLE_COLLISION_ENGINE

#define __SIMPLE_COLLISION_ENGINE
#include "collision_engine.hpp"
#include "vertex.hpp"
#include "edge.hpp"

class simple_collision_engine : public collision_engine {
public:

   simple_collision_engine(environment * env) : collision_engine(env) {}

   void recalibrate() {
   }

   bool is_collision(edge * e) {
      return is_collision(e->from,e->to);
   }

   bool is_collision(const vertex & from, const vertex & to) {
      if (to.x < 0 || to.x >= env->xsize || to.y < 0 || to.y >= env->ysize)
         return true;
      if (from.x < 0 || from.x >= env->xsize || from.y < 0 || from.y >= env->ysize)
         return true;
      if (env->isset(from) || env->isset(to))
         return true;
      return false;
   }
};

#endif
