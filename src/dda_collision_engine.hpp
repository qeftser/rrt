
#ifndef __DDA_COLLISION_ENGINE

#define __DDA_COLLISION_ENGINE
#include "collision_engine.hpp"
#include "vertex.hpp"
#include "edge.hpp"

/*
 * Based on:
 * https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
 */

class dda_collision_engine : public collision_engine {
public:

   dda_collision_engine(environment * env) : collision_engine(env) {}

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

      vertex distance = to - from;
      double step = fabs(distance.x);
      if (step < fabs(distance.y))
         step = fabs(distance.y);

      distance = distance/(step);
      vertex position = from;

      int i = 0;
      while (i <= step) {
         if (env->isset(position))
            return true;
         position += distance;
         ++i;
      }

      return false;
   }
};

#endif
