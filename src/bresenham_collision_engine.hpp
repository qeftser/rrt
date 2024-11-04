
#ifndef __BRESENHAM_COLLISION_ENGINE

#define __BRESENHAM_COLLISION_ENGINE
#include "collision_engine.hpp"
#include "vertex.hpp"
#include "edge.hpp"

/*
 * Based on:
 * https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
 */

#error broken

class bresenham_collision_engine : public collision_engine {
public:

   bresenham_collision_engine(environment * env) : collision_engine(env) {}

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
      /*
      if (env->isset(from) || env->isset(to))
         return true;
         */
      env->occupancy[(int)floor(from.x)][(int)floor(from.y)] = 1;
      env->occupancy[(int)floor(to.x)][(int)floor(to.y)] = 1;

      int x0 = (from.x < to.x ? from.x : to.x);
      int x1 = (from.x > to.x ? from.x : to.x);
      int y0 = (from.y < to.y ? from.y : to.y);
      int y1 = (from.y > to.y ? from.y : to.y);

      int dx = x1 - x0;
      int dy = y1 - y0;
      int D = 2*dy - dx;
      int y = y0;

      for (int x = x0; x <= x1; ++x) {
         if (env->occupancy[x][y])
            return true;
         if (D > 0) {
            y = y+1;
            D = D - 2*dx;
         }
         D = D + 2*dy;
      }

      return false;
   }
};

#endif
