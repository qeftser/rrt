
#ifndef __SST

#define __SST
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "collision_engine.hpp"
#include "bin_point_set.hpp"
#include "rrt_base.hpp"
#include <cfloat>

class sst : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   bin_point_set witness_list;

   const double _Obn = 1.5;
   const double _Os  = 0.75;

   sst() : env(NULL), points(NULL), ce(NULL),
           witness_list(bin_point_set(5)) {}
   sst(const vertex init, environment * env, point_set * p, collision_engine * c) 
      : env(env), points(p), ce(c), witness_list(bin_point_set(5)) {
      edge * start = new edge(init,init);
      points->add(start);
      witness_list.add(new edge(init,init,start));
   }

   ~sst() {
      witness_list.reset();
   }

   void restart(const vertex pos) {
      points->reset();
      witness_list.reset();
      edge * start = new edge(pos,pos);
      points->add(start);
      witness_list.add(new edge(pos,pos,start));
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         
         /* new sample */
         vertex random = vertex::rand(env->xsize,env->ysize);

         /* try to find nearby points *before* adjusting */
         edge * nearest = NULL;
         std::vector<edge *> near;
         points->in_range(random,_Obn,&near);

         if (!near.empty()) {
            double low_cost = DBL_MAX;
            for (edge * e : near) {
               if (!ce->is_collision(e->to,random) &&
                   e->cost+e->to.dist(random) < low_cost) {
                  low_cost = e->cost+e->to.dist(random);
                  nearest = e;
               }
            }
         }
         else
            nearest = points->closest(random);

         if (nearest && !ce->is_collision(nearest->to,random)) {
            if (nearest->to.dist(random) >= _Obn) {
               random = random-nearest->to;
               random = ((random/random.dist(vertex()))*((double)rand()/RAND_MAX))+nearest->to; // monte-carlo prop
            }

            edge * new_edge = new edge(nearest->to,random,nearest);
            new_edge->cost = nearest->cost+nearest->to.dist(random);

            edge * witness = witness_list.closest(new_edge->to);

            if (witness->to.dist(new_edge->to) > _Os) {
               edge * new_witness = new edge(new_edge->to,new_edge->to,new_edge);
               witness_list.add(new_witness);
               nearest->children.push_back(new_edge);
               points->add(new_edge);
            }
            else {
               if (witness->parent->cost < new_edge->cost) {
                  delete new_edge;
               }
               else {
                  auto result = std::remove(witness->parent->parent->children.begin(),
                                            witness->parent->parent->children.end(),
                                            witness->parent);
                  witness->parent->parent->children.erase(result,
                        witness->parent->parent->children.end());
                  swap_parents(witness->parent,new_edge);
                  points->remove(witness->parent);
                  delete witness->parent;
                  witness->parent = new_edge;
                  nearest->children.push_back(new_edge);
                  points->add(new_edge);
               }
            }
         }
      }
   }

private:

   inline void swap_parents(edge * old, edge * nev) {
      for (edge * e : old->children) {
         e->parent = nev;
         e->from = nev->to;
         nev->children.push_back(e);
      }
      old->children.clear();
   }

};

#endif
