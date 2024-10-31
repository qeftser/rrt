
#ifndef __POINT_SET

#define __POINT_SET
#include "vertex.hpp"
#include "edge.hpp"
#include <vector>

class point_set {
public:

   virtual edge * closest(const vertex & v) = 0;
   virtual void in_range(const vertex & v, double range, std::vector<edge *> * results) = 0;
   virtual void add(edge * e) = 0;
   virtual void remove(edge * e) = 0;

   virtual void draw(sf::RenderWindow * window) = 0;

};

#endif
