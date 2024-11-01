
#ifndef __VERTEX

#define __VERTEX
#include <cmath>
#include <cstdlib>

class vertex {
public:
   double x;
   double y;

   vertex() : x(0), y(0) {}
   vertex(const vertex & other) : x(other.x), y(other.y) {}
   vertex(double x, double y) : x(x), y(y) {}

   const double dist(const vertex & other) const {
      return std::sqrt((other.x-x)*(other.x-x)+(other.y-y)*(other.y-y));
   }

   vertex & operator+=(const vertex & other) {
      x += other.x;
      y += other.y;
      return *this;
   }

   vertex operator+(const vertex & other) {
      return vertex(x+other.x,y+other.y);
   }

   vertex operator-(const vertex & other) {
      return vertex(x-other.x,y-other.y);
   }

   vertex operator/(double divisor) {
      return vertex(x/divisor,y/divisor);
   }

   vertex operator*(double multiplier) {
      return vertex(x*multiplier,y*multiplier);
   }

   static vertex rand(int max_x, int max_y) {
      return vertex(((double)std::rand()/RAND_MAX)*max_x,((double)std::rand()/RAND_MAX)*max_y);
   }

   double slope(const vertex & other) const {
      return ((x-other.x)/(y-other.y));
   }
};

#endif
