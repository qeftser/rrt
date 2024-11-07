
#ifndef __BIN_POINT_SET

#define __BIN_POINT_SET
#include "point_set.hpp"
#include <cfloat>
#include <unordered_map>

class bin_point_set : public point_set {
public:
   
   struct point {
      int x;
      int y;
   };

   struct occupant {
      point pos;
      std::vector<edge *> edges;
   };

   int divisor;
   std::vector<occupant *> occupancy;
   std::unordered_map<long,occupant *> apartments;

   bin_point_set(int divisor) : divisor(divisor) {};

   edge * closest(const vertex & v) {

      /* same as simple_point_set */
      edge * closest_edge = NULL;
      double min_distance = DBL_MAX;

      /* abstraction of collections of points into 
       * occupancy groups                          */
      point closest_occupant;
      double min_occupant = DBL_MAX;
      point pos = trans(v);

      /* get closest occupancy group to point */
      for (occupant * o : occupancy) {
         if (dist(&o->pos,&pos) < min_occupant) {
            min_occupant = dist(&o->pos,&pos);
            closest_occupant = o->pos;
         }
      }

      /* get closest value among the occupancy points, given that
       * the occupancy points exist.
       * This allows us to avoid computing about 90% of the distance
       * calls if the map is saturated. Otherwise, speed should be comparable */
      for (int i = closest_occupant.x-1; i < closest_occupant.x+2; ++i) {
         for (int j = closest_occupant.y-1; j < closest_occupant.y+2; ++j) {
            if (apartments.count(i|((long)j<<32))) {
               occupant * room = apartments.at(i|((long)j<<32));
               for (edge * e : room->edges) {
                  if (e->to.dist(v) < min_distance) {
                     min_distance = e->to.dist(v);
                     closest_edge = e;
                  }
               }
            }
         }
      }

      return closest_edge;
   }

   void in_range(const vertex & v, double range, std::vector<edge *> * results) {

      /* find the closest occupancy */
      point closest_occupant;
      double min_occupant = DBL_MAX;
      point pos = trans(v);
      for (occupant * o : occupancy) {
         if (dist(&o->pos,&pos) < min_occupant) {
            min_occupant = dist(&o->pos,&pos);
            closest_occupant = o->pos;
         }
      }

      /* the difference from closest is that we need to check all
       * occupancies that may contain a point in the range. This means
       * we may need to check more than 9 bins                         */
      int room_range = ceil(range/divisor);
      for (int i = closest_occupant.x-range; i <= closest_occupant.x+range; ++i) {
         for (int j = closest_occupant.y-range; j <= closest_occupant.y+range; ++j) {
            if (apartments.count(i|((long)j<<32))) {
               occupant * room = apartments.at(i|((long)j<<32));
               for (edge * e : room->edges) {
                  if (e->to.dist(v) <= range || e->from.dist(v) <= range)
                     results->push_back(e);
               }
            }
         }
      }
   }

   void add(edge * e) {
      point pos = trans(e->to);
      if (!apartments.count(pos.x|((long)pos.y<<32))) {
         apartments[pos.x|((long)pos.y<<32)] = (occupant *)calloc(sizeof(occupant),1);
         occupancy.push_back(apartments[pos.x|((long)pos.y<<32)]);
         apartments[pos.x|((long)pos.y<<32)]->pos = pos;
         apartments[pos.x|((long)pos.y<<32)]->edges = std::vector<edge *>();
      }
      apartments[pos.x|((long)pos.y<<32)]->edges.push_back(e);
   }

   void remove(edge * e) {
      point pos = trans(e->to);
      std::vector<edge *> * room = &apartments[pos.x|((long)pos.y<<32)]->edges;
      auto iterator = std::remove(room->begin(),room->end(),e);
      room->erase(iterator,room->end());
      if (room->empty()) {
         auto iterator = std::remove(occupancy.begin(),occupancy.end(),apartments[pos.x|((long)pos.y<<32)]);
         occupancy.erase(iterator,occupancy.end());
         apartments.erase(pos.x|((long)pos.y<<32));
         free(apartments[pos.x|((long)pos.y<<32)]);
      }
   }

   void draw(sf::RenderWindow * window) {
      for (occupant * o : occupancy)
         for (edge * e : o->edges)
            e->display_edge(window);
   }

   void start(edge * e) {
      start_point = e;
      add(start_point);
   }

   void reset() {
      remove(start_point);
      for (occupant * o : occupancy) {
         for (edge * e : o->edges) {
            delete e;
         }
         free(o);
      }
      occupancy.clear();
      apartments.clear();
      add(start_point);
   }

private:

   inline double const dist(const point * p1, const point * p2) const {
      return sqrt((double)((p1->x-p2->x)*(p1->x-p2->x)+(p1->y-p2->y)*(p1->y-p2->y)));
   }

   inline point const trans(const vertex & v) const {
      return point{(int)(v.x/divisor),(int)(v.y/divisor)};
   }
   
};

#endif
