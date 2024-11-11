
#ifndef __SST_X

#define __SST_X
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "collision_engine.hpp"
#include "bin_point_set.hpp"
#include "rrt_base.hpp"
#include <cfloat>
#include <queue>
#include <vector>
#include <cfloat>
#include <unordered_set>

class sst_x : public rrt_base {
public:

   const double R = 1.0;          // range to check for edges to adopt
   const double cull_range = 4.5; // range to cull edges when blocked by obstacles
   const double _Obn = 1.5;       // max edge length
   const double _Os  = 1.25;       // diameter of dominance regions

   environment * env;
   point_set * points;
   collision_engine * ce;
   bin_point_set orphanage;
   bin_point_set witness_list;

   std::queue<edge *> Q;

   sst_x() : env(NULL), points(NULL), ce(NULL),
             orphanage(bin_point_set(5)),
             witness_list(bin_point_set(5)) {}
   sst_x(const vertex init, environment * env, point_set * p, collision_engine * c)
      : env(env), points(p), ce(c), orphanage(bin_point_set(5)), witness_list(bin_point_set(5)) {
         edge * start = new edge(init,init);
         points->add(start);
         witness_list.add(new edge(init,init,start));
      }

   ~sst_x() {
      orphanage.reset();
      witness_list.reset();
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         /* new sample */
         vertex random = vertex::rand(env->xsize,env->ysize);

         /* try to find nearby points *before* adjusting */
         std::vector<edge *> near;
         points->in_range(random,_Obn,&near);

         edge * nearest = get_parent(random,&near);

         if (nearest) {
            if (nearest->to.dist(random) >= _Obn) {
               random = random-nearest->to;
               random = ((random/random.dist(vertex()))*(((double)rand()/RAND_MAX)*_Obn))+nearest->to; // monte-carlo prop
            }

            edge * new_edge = new edge(nearest->to,random,nearest);
            new_edge->cost = nearest->cost+nearest->to.dist(random);

            edge * witness = witness_list.closest(new_edge->to);

            if (witness->parent->to.dist(new_edge->to) > _Os) {
               edge * new_witness = new edge(new_edge->to,new_edge->to,new_edge);
               witness_list.add(new_witness);
               nearest->children.push_back(new_edge);
               points->add(new_edge);
            }
            else {
               if (witness->parent->cost < new_edge->cost) {
                  delete new_edge;
                  if (witness->parent->orphan)
                     reconstruct_graph(witness->parent);
               }
               else {
                  bool propagate = false;
                  if (!witness->parent->orphan) {
                     auto result = std::remove(witness->parent->parent->children.begin(),
                                               witness->parent->parent->children.end(),
                                               witness->parent);
                     witness->parent->parent->children.erase(result,
                           witness->parent->parent->children.end());
                     swap_parents(witness->parent,new_edge);
                     points->remove(witness->parent);
                     propagate = true;
                  }
                  else 
                     orphanage.remove(witness->parent);
                  delete witness->parent;

                  witness_list.remove(witness);
                  witness->parent = new_edge;
                  witness->to = new_edge->to;
                  witness_list.add(witness);

                  nearest->children.push_back(new_edge);
                  points->add(new_edge);
                  if (propagate) {
                     near.clear();
                     orphanage.in_range(new_edge->to,R,&near);
                     for (edge * e : near)
                        reconstruct_graph(e);
                  }
               }
            }
         }
      }
   }

   void notify_obstacle(const vertex pos) {
      std::vector<edge *> near;
      points->in_range(pos,cull_range,&near);
      for (edge * e : near) {
         if (ce->is_collision(e->from,e->to)) {
            if (e->parent) {
               auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
               e->parent->children.erase(result,e->parent->children.end());
               e->parent = NULL;
            }
            propagate_orphanhood(e);
         }
      }
      orphanage.in_range(pos,cull_range,&near);
restart:
      for (edge * e : near) {
         if (e->orphan) {
            orphanage.remove(e);
            auto iterator = remove(near.begin(),near.end(),e);
            near.erase(iterator,near.end());
            edge * witness = witness_list.closest(e->to);
            if (witness->parent == e) {
               witness_list.remove(witness);
               delete witness; // no witnesses!
            }
            delete e;
            goto restart;
         }
      }
      for (edge * e : near) {
         std::vector<edge *> local_near;
         orphanage.in_range(e->to,R,&local_near);
         for (edge * w : local_near) {
            if (w->orphan)
               continue;
            reconstruct_graph(w);
         }
      }
   }

   void restart(const vertex pos) {
      points->reset();
      orphanage.reset();
      witness_list.reset();
      edge * start = new edge(pos,pos);
      points->add(start);
      witness_list.add(new edge(pos,pos,start));
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

   inline void reconstruct_graph(edge * orphan) {
      if (!orphan->orphan)
         return;
      std::unordered_set<long> seen;
      Q.push(orphan);
      seen.insert((long)orphan);
      while (!Q.empty()) {
         orphan = Q.front(); Q.pop();
         std::vector<edge *> near;
         points->in_range(orphan->to,R,&near);
         edge * canidate = get_parent(orphan->to,&near);
         if (canidate && orphan->to.dist(canidate->to) < _Obn) {
            orphan->parent = canidate;
            canidate->children.push_back(orphan);
            orphan->orphan = false;
            orphan->from = canidate->to;
            orphan->cost = canidate->cost+canidate->to.dist(orphan->to);
            orphanage.remove(orphan);
            points->add(orphan);
            near.clear();
            orphanage.in_range(orphan->to,R,&near);
            for (edge * e : near) {
               if (e->orphan && !seen.count((long)e)) {
                  Q.push(e); seen.insert((long)e);
               }
            }
         }
      }
   }

   inline void propagate_orphanhood(edge * orphan) {
      Q.push(orphan);
      while (!Q.empty()) {
         orphan = Q.front(); Q.pop();
         if (orphan->orphan)
            continue;
         orphan->orphan = true;
         points->remove(orphan);
         orphanage.add(orphan);
         orphan->parent = NULL;
         orphan->cost = DBL_MAX;
         for (edge * w : orphan->children) {
            Q.push(w);
         }
         orphan->children.clear();
      }
   }

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
