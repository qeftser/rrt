
#ifndef __SIMPLE_POINT_SET

#define __SIMPLE_POINT_SET
#include "point_set.hpp"
#include <cfloat>

class simple_point_set : public point_set {
public:
   std::vector<edge *> edges;

   edge * closest(const vertex & v) {
      edge * closest_edge = NULL;
      double min_distance = DBL_MAX;
      for (edge * e : edges) {
         if (e->to.dist(v) < min_distance) {
            min_distance = e->to.dist(v);
            closest_edge = e;
         }
      }
      return closest_edge;
   }

   void in_range(const vertex & v, double range, std::vector<edge *> * results) {
      for (edge * e : edges)
         if (e->to.dist(v) <= range)
            results->push_back(e);
   }

   void add(edge * e) {
      edges.push_back(e);
   }

   void remove(edge * e) {
      auto iterator = std::remove(edges.begin(),edges.end(),e);
      edges.erase(iterator,edges.end());
   }

   void draw(sf::RenderWindow * window) {
      for (edge * e : edges)
         edge::display_edge(window,e);
   }

   void start(edge e) {
      start_point = e;
      add(&start_point);
   }

   void reset() {
      remove(&start_point);
      for (edge * e : edges)
         delete e;
      edges.clear();
      add(&start_point);
   }

};

#endif
