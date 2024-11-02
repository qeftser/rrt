
#ifndef __XIAOLIN_COLLISION_ENGINE

#define __XIAOLIN_COLLISION_ENGINE
#include "collision_engine.hpp"
#include "vertex.hpp"
#include "edge.hpp"

#error !!xiaolin_collision_engine will crash at boundries!!

/*
 * Based on:
 * https://en.wikipedia.org/wiki/Xiaolin_Wu%27s_line_algorithm 
 */

class xiaolin_collision_engine : public collision_engine {
public:
   xiaolin_collision_engine(environment * env) : collision_engine(env) {}

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

      bool steep = fabs(to.y - from.y) > fabs(to.x - from.x);

      double x0 = from.x;
      double y0 = from.y;
      double x1 = to.x;
      double y1 = to.y;

      double interm;
      if (steep) {
         interm = x0;
         x0 = y0;
         y0 = interm;
         interm = x1;
         x1 = y1;
         y1 = interm;
      }
      if (x0 > x1) {
         interm = x0;
         x0 = x1;
         x1 = interm;
         interm = y0;
         y0 = y1;
         y1 = interm;
      }

      double dx = x1 - x0;
      double dy = y1 - y0;

      double gradient;
      if (dx == 0.0)
         gradient = 1;
      else
         gradient = dy / dx;

      // handle first endpoint
      double xend = round(x0);
      double yend = y0 + gradient * (xend - x0);
      double xgap = rfpart(x0 + 0.5);
      int xpxl1 = xend;
      int ypxl1 = ipart(yend);
      if (steep) {
         if (env->occupancy[ypxl1][xpxl1])
            return true;
         if (env->occupancy[ypxl1+1][xpxl1])
            return true;
      }
      else {
         if (env->occupancy[ypxl1][xpxl1])
            return true;
         if (env->occupancy[ypxl1][xpxl1+1])
            return true;
      }
      double intery = yend + gradient;

      // handle second endpoint
      xend = round(x1);
      yend = y1 + gradient * (xend - x1);
      xgap = fpart(x1 + 0.5);
      int xpxl2 = xend;
      int ypxl2 = ipart(yend);
      if (steep) {
         if (env->occupancy[ypxl2][xpxl2])
            return true;
         if (env->occupancy[ypxl2+1][xpxl2])
            return true;
      }
      else {
         if (env->occupancy[ypxl2][xpxl2])
            return true;
         if (env->occupancy[ypxl2][xpxl2+1])
            return true;
      }

      if (steep) {
         for (int x = xpxl1 + 1; x <= xpxl2 - 1; ++x) {
            if (env->occupancy[ipart(intery)][x])
               return true;
            if (env->occupancy[ipart(intery)+1][x])
               return true;
            intery += gradient;
         }
      }
      else {
         for (int x = xpxl1 + 1; x <= xpxl2 - 1; ++x) {
            if (env->occupancy[x][ipart(intery)])
               return true;
            if (env->occupancy[x][ipart(intery)+1])
               return true;
         }
      }

      return false;
   }

private:
   inline int const ipart(const double x) const { return (int)floor(x); }
   inline double const fpart(const double x) const { return x - ipart(x); }
   inline double const rfpart(const double x) const { return 1 - fpart(x); }
};

#endif
