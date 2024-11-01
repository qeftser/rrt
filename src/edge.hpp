
#ifndef __EDGE

#define __EDGE
#include "vertex.hpp"
#include "environment.hpp"
#include <vector>
#include <SFML/Graphics.hpp>

class edge {
public:
   vertex from;
   vertex to;
   edge * parent;
   std::vector<edge *> children;

   edge() : children(std::vector<edge *>()), parent(NULL) {}
   edge(vertex from, vertex to) 
      : children(std::vector<edge *>()), parent(NULL), from(from), to(to) {}
   edge(vertex from, vertex to, edge * parent) 
      : children(std::vector<edge *>()), parent(parent), from(from), to(to) {}

   static void display_edge(sf::RenderWindow * w, edge * e) {
      sf::Vertex points[2];
      points[0].position = sf::Vector2f(e->from.x*10,e->from.y*10);
      points[1].position = sf::Vector2f(e->to.x*10,e->to.y*10);
      w->draw(points,2,sf::Lines);
   }
};

#endif
