
#ifndef __HYPERBIN_POINT_SET

#define __HYPERBIN_POINT_SET
#include "point_set.hpp"
#include <cfloat>
#include <unordered_map>

class hyperbin_point_set : public point_set {
public:

   struct point {
      int x;
      int y;
   };

   struct occupant {
      point pos;
      std::vector< edge *> edges;
   };

   int divisor;
   std::vector<occupant *> occupancy;
   std::unordered_map<long,occupant *> apartments;

   hyperbin_point_set(int divisor) : divisor(divisor) {};

   edge * closest(const vertex & v) {
      
      edge * closest_edge = NULL;
      double min_distance = DBL_MAX;

      point closest_occupant;
      double min_occupant = DBL_MAX;
      point pos = trans(v);

      if (!apartments.count(pos.x|((long)pos.y<<32))) {
         int range = 1;
         while (true) {
            int jl = pos.y-range;
            int jh = pos.y+range;
            int il = pos.x-range;
            int ih = pos.x+range;
            for (int i = il; i <= ih; ++i) {
               if (apartments.count(i|((long)jl<<32))) {
                  occupant * room = apartments.at(i|((long)jl<<32));
                  for (edge * e : room->edges) {
                     if (e->to.dist(v) < min_distance) {
                        min_distance = e->to.dist(v);
                        closest_edge = e;
                     }
                  }
               }
               if (apartments.count(i|((long)jh<<32))) {
                  occupant * room = apartments.at(i|((long)jh<<32));
                  for (edge * e : room->edges) {
                     if (e->to.dist(v) < min_distance) {
                        min_distance = e->to.dist(v);
                        closest_edge = e;
                     }
                  }
               }
            }
            for (int j = jl+1; j <= jh-1; ++j) {
               if (apartments.count(il|((long)j<<32))) {
                  occupant * room = apartments.at(il|((long)j<<32));
                  for (edge * e : room->edges) {
                     if (e->to.dist(v) < min_distance) {
                        min_distance = e->to.dist(v);
                        closest_edge = e;
                     }
                  }
               }
               if (apartments.count(ih|((long)j<<32))) {
                  occupant * room = apartments.at(ih|((long)j<<32));
                  for (edge * e : room->edges) {
                     if (e->to.dist(v) < min_distance) {
                        min_distance = e->to.dist(v);
                        closest_edge = e;
                     }
                  }
               }
            }
            if (closest_edge)
               return closest_edge;
            ++range;
         }
      }


      int selector = (v.x - pos.x < 0.5 ? 0 : 1);
      selector |= (v.y - pos.y < 0.5 ? 0 : 2);

      switch (selector) {
         case 0:
            {
               for (int i = pos.x-1; i <= pos.x; ++i) {
                  for (int j = pos.y-1; j <= pos.y; ++j) {
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
            }
         case 1:
            {
               for (int i = pos.x; i <= pos.x+1; ++i) {
                  for (int j = pos.y-1; j <= pos.y; ++j) {
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
            }
         case 2:
            {
               for (int i = pos.x-1; i <= pos.x; ++i) {
                  for (int j = pos.y; j <= pos.y+1; ++j) {
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
            }
         case 3:
            {
               for (int i = pos.x; i <= pos.x+1; ++i) {
                  for (int j = pos.y; j <= pos.y+1; ++j) {
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
            }
      }
      return closest_edge;
   }

   void in_range(const vertex & v, double range, std::vector<edge *> * results) {

      point pos = trans(v);

      /* the difference from closest is that we need to check all
       * occupancies that may contain a point in the range. This means
       * we may need to check more than 9 bins                         */
      int room_range = ceil(range/divisor);
      for (int i = pos.x-range; i <= pos.x+range; ++i) {
         for (int j = pos.y-range; j <= pos.y+range; ++j) {
            if (apartments.count(i|((long)j<<32))) {
               occupant * room = apartments.at(i|((long)j<<32));
               for (edge * e : room->edges) {
                  if (e->to.dist(v) <= range || e->from.dist(v) <= range)
                     results->push_back(e);
               }
            }
         }
      }

      /*
      /* the difference from closest is that we need to check all
       * occupancies that may contain a point in the range. This means
       * we may need to check more than 9 bins                         
      int room_range = ceil(range/divisor);
      for (int i = pos.x-range+1; i < pos.x+range; ++i) {
         for (int j = pos.y-range+1; j < pos.y+range; ++j) {
            if (apartments.count(i|((long)j<<32))) {
               occupant * room = apartments.at(i|((long)j<<32));
               for (edge * e : room->edges) {
                  results->push_back(e);
               }
            }
         }
      }
      for (int i = pos.x-range; i <= pos.x+range; ++i) {
         if (apartments.count(i|((long)(pos.y-range)<<32))) {
            occupant * room = apartments.at(i|((long)(pos.y-range)<<32));
            for (edge * e : room->edges) {
               if (e->to.dist(v) <= range || e->from.dist(v) <= range) {
                  results->push_back(e);
               }
            }
         }
         if (apartments.count(i|((long)(pos.y+range)<<32))) {
            occupant * room = apartments.at(i|((long)(pos.y+range)<<32));
            for (edge * e : room->edges) {
               if (e->to.dist(v) <= range || e->from.dist(v) <= range) {
                  results->push_back(e);
               }
            }
         }
      }
      for (int j = pos.y-range+1; j < pos.y+range; ++j) {
         if (apartments.count((long)(pos.x+range)|((long)j<<32))) {
            occupant * room = apartments.at((long)(pos.x+range)|((long)j<<32));
            for (edge * e : room->edges) {
               if (e->to.dist(v) <= range || e->from.dist(v) <= range) {
                  results->push_back(e);
               }
            }
         }
         if (apartments.count((long)(pos.x-range)|((long)j<<32))) {
            occupant * room = apartments.at((long)(pos.x-range)|((long)j<<32));
            for (edge * e : room->edges) {
               if (e->to.dist(v) <= range || e->from.dist(v) <= range) {
                  results->push_back(e);
               }
            }
         }
      }
      */
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
         free(apartments[pos.x|((long)pos.y<<32)]);
         apartments.erase(pos.x|((long)pos.y<<32));
      }
   }

   void draw(sf::RenderWindow * window) {
      for (occupant * o : occupancy)
         for (edge * e : o->edges)
            e->display_edge(window);
   }

   void reset() {
      for (occupant * o : occupancy) {
         for (edge * e : o->edges) {
            delete e;
         }
         free(o);
      }
      occupancy.clear();
      apartments.clear();
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
