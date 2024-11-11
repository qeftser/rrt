
#ifndef __SST_FN

#define __SST_FN
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "bin_point_set.hpp"
#include "collision_engine.hpp"
#include "rrt_base.hpp"
#include <cfloat>
#include <queue>
#include <vector>
#include <unordered_set>

class sst_fn : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   std::unordered_set<long> infertile;
   bin_point_set witness_list;

   const double _Obn = 1.5;
   const double _Os = 0.75;
   const int fixed_limit = 25000;
   int count = 0;

   sst_fn() : env(NULL), points(NULL), ce(NULL), witness_list(bin_point_set(5)) {}
   sst_fn(const vertex init, environment * env, point_set * p, collision_engine * c)
      : env(env), points(p), ce(c), witness_list(bin_point_set(5)) {
      edge * start = new edge(init,init);
      points->add(start);
      witness_list.add(new edge(init,init,start));
      infertile.insert((long)start);
      ++count;
   }

   ~sst_fn() {
      witness_list.reset();
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);

         std::vector<edge *> near;
         points->in_range(random,_Obn,&near);
         edge * nearest = get_parent(random,&near);

         if (nearest) {
            if (nearest->to.dist(random) >= _Obn) {
               random = random-nearest->to;
               random = ((random/random.dist(vertex()))*(((double)rand()/RAND_MAX)*_Obn))+nearest->to;
            }

            edge * new_edge = new edge(nearest->to,random,nearest);
            new_edge->cost = nearest->cost+nearest->to.dist(random);

            edge * witness = witness_list.closest(new_edge->to);

            if (witness->parent->to.dist(new_edge->to) > _Os) {
               edge * new_witness = new edge(new_edge->to,new_edge->to,new_edge);
               witness_list.add(new_witness);
               if (nearest->children.empty())
                  infertile.erase((long)nearest);
               nearest->children.push_back(new_edge);
               points->add(new_edge);
               infertile.insert((long)new_edge);
               ++count;

               if (count > fixed_limit) {
                  edge * sacrifice = random_sacrifice();
                  infertile.erase((long)sacrifice);
                  --count;
                  auto result = remove(sacrifice->parent->children.begin(),sacrifice->parent->children.end(),sacrifice);
                  sacrifice->parent->children.erase(result,sacrifice->parent->children.end());
                  points->remove(sacrifice);
                  if (sacrifice->parent->children.empty())
                     infertile.insert((long)sacrifice->parent);
                  edge * witness = witness_list.closest(sacrifice->to);
                  if (witness->parent == sacrifice) {
                     witness_list.remove(witness);
                     delete witness; // no witnesses!
                  }
                  delete sacrifice;
               }
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
                  if (witness->parent->parent->children.empty())
                     infertile.insert((long)witness->parent->parent);
                  swap_parents(witness->parent,new_edge);
                  if (new_edge->children.empty()) {
                     infertile.erase((long)witness->parent);
                     infertile.insert((long)new_edge);
                  }
                  points->remove(witness->parent);
                  delete witness->parent;

                  witness_list.remove(witness);
                  witness->parent = new_edge;
                  witness->to = new_edge->to;
                  witness_list.add(witness);

                  if (nearest->children.empty())
                     infertile.erase((long)nearest);
                  nearest->children.push_back(new_edge);
                  points->add(new_edge);
               }
            }
         }
      }
   }

   void restart(const vertex pos) {
      points->reset();
      witness_list.reset();
      infertile.clear();
      edge * start = new edge(pos,pos);
      points->add(start);
      witness_list.add(new edge(pos,pos,start));
      infertile.insert((long)start);
      count = 1;
   }

private:

   inline edge * get_parent(const vertex & v, std::vector<edge *> * near) {
      double low_cost = DBL_MAX;
      edge * low_edge = NULL;
      if (!near->empty()) {
         for (edge * e : *near) {
            if (!ce->is_collision(e->to,v) &&
                e->cost+e->to.dist(v) < low_cost) {
               low_cost = e->cost+e->to.dist(v);
               low_edge = e;
            }
         }
      }
      else {
         low_edge = points->closest(v);
         if (ce->is_collision(low_edge->to,v))
            return NULL;
      }
      return low_edge;
   }

   inline void swap_parents(edge * old, edge * nev) {
      for (edge * e : old->children) {
         e->parent = nev;
         e->from = nev->to;
         nev->children.push_back(e);
      }
      old->children.clear();
   }

   inline edge * random_sacrifice() {
      int idx = rand()%infertile.size();
      auto it = infertile.begin();
      for (int i = 0; i < idx; ++i)
         ++it;
      return (edge *)*it;
   }

};


#endif
