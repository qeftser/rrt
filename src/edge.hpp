
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

   bool is_collision(environment * env) {
      vertex step((to.x-from.x)/10000,(to.y-from.y)/10000);
      vertex pos(from.x,from.y);
      while (to.dist(pos) > 0.25) {
         if (env->occupancy[(int)pos.x][(int)pos.y])
            return true;
         pos += step;
      }
      return false;
   }

   static bool is_collision(const vertex & from, const vertex & to, environment * env) {
      vertex step(1 / (to.x-from.x),1 / (to.y-from.y));
      vertex pos(from.x,from.y);
      while (to.dist(pos) > 0.25) {
         if (pos.x < 0 || pos.x > env->xsize || pos.y < 0 || pos.x > env->ysize || env->occupancy[(int)pos.x][(int)pos.y])
            return true;
         pos += step;
      }
      return false;
   }
};

#endif
