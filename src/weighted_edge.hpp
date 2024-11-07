
#ifndef __WEIGHTED_EDGE

#define __WEIGHTED_EDGE
#include "edge.hpp"

class weighted_edge : public edge {
public:
   double cost;
   bool orphan;

   weighted_edge() : edge(), orphan(false), cost(0) {}
   weighted_edge(vertex from, vertex to)
      : edge(from,to), orphan(false), cost(0) {}
   weighted_edge(vertex from, vertex to, edge * parent)
      : edge(from,to,parent), orphan(false), cost(0) {}

   virtual void display_edge(sf::RenderWindow * w) {
      if (orphan)
         return;
      sf::Vertex points[2];
      points[0].position = sf::Vector2f(from.x*10,from.y*10);
      points[1].position = sf::Vector2f(to.x*10,to.y*10);
      w->draw(points,2,sf::Lines);
   }
};

#endif
