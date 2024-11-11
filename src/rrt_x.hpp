
#ifndef __RRT_X

#define __RRT_X
#include "vertex.hpp"
#include "environment.hpp"
#include "rrt_base.hpp"
#include "bin_point_set.hpp"
#include "collision_engine.hpp"
#include <queue>
#include <vector>
#include <cfloat>
#include <unordered_set>

class rrt_x : public rrt_base {
public:

   const double R = 1.0;
   const double cull_range = 4.5;

   environment * env;
   point_set * points;
   collision_engine * ce;
   bin_point_set orphanage;

   std::queue<edge *> Q;

   rrt_x() : env(NULL), points(NULL), ce(NULL), 
             orphanage(bin_point_set(5)) {}
   rrt_x(const vertex init, environment * env, point_set * p, collision_engine * c) 
      : env(env), points(p), ce(c), orphanage(bin_point_set(5)) {
      points->add(new edge(init,init));
   }

   ~rrt_x() {
      orphanage.reset();
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         edge * nearest = points->closest(random);

         /* state change computation
          * this is where we can extend to arbitrary DOF
          */
         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;

         /* get near vertices */
         std::vector<edge *> near;
         near.push_back(nearest);
         points->in_range(new_state,R,&near);

         /* find lowest cost for parent */
         edge * low_edge = get_parent(new_state,&near);

         /* insert new edge */
         if (low_edge) {
            edge * new_edge = new edge(low_edge->to,new_state,low_edge);
            new_edge->cost = low_edge->cost+low_edge->to.dist(new_state);
            low_edge->children.push_back(new_edge);
            points->add(new_edge);

            /* find any children to rewire */
            orphanage.in_range(new_state,R,&near);
            rewire_neighbors(new_edge,&near);
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
            delete e;
            goto restart;
         }
      }
      for (edge * e : near) {
         std::vector<edge *> local_near;
         points->in_range(e->to,R,&local_near);
         orphanage.in_range(e->to,R,&local_near);
         rewire_neighbors(e,&local_near);
      }
   }

   void restart(const vertex pos) {
      points->reset();
      orphanage.reset();
      points->add(new edge(pos,pos));
   }

private:

   inline edge * get_parent(const vertex & v, std::vector<edge *> * near) {
      double low_cost = DBL_MAX;
      edge * low_edge = NULL;
      for (edge * e : *near) {
         if (!e->orphan &&
             !ce->is_collision(e->to,v) &&
             e->cost+e->to.dist(v) < low_cost) {
            low_cost = e->cost+e->to.dist(v);
            low_edge = e;
         }
      }

      return low_edge;
   }

   inline void rewire_neighbors(edge * self, std::vector<edge *> * near) {
      for (edge * e : *near) {
         if (!ce->is_collision(self->to,e->to) &&
             self->cost+self->to.dist(e->to) < e->cost) {
            if (e->parent) {
               auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
               e->parent->children.erase(result,e->parent->children.end());
            }
            e->parent = self;
            e->from = self->to;
            self->children.push_back(e);
            e->cost = self->cost+self->to.dist(e->to);
            if (e->orphan) 
               adopt(e);
         }
      }
   }

   inline void adopt(edge * orphan) {
      std::unordered_set<long> seen;
      Q.push(orphan);
      seen.insert((long)orphan);
      while (!Q.empty()) {
         orphan = Q.front(); Q.pop();
         std::vector<edge *> near;
         orphan->orphan = false;
         orphanage.remove(orphan);
         points->add(orphan);
         points->in_range(orphan->to,R,&near);
         orphanage.in_range(orphan->to,R,&near);
         for (edge * e : near) {
            if (!ce->is_collision(orphan->to,e->to) &&
                orphan->cost+orphan->to.dist(e->to) < e->cost) {
               if (e->parent) {
                  auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
                  e->parent->children.erase(result,e->parent->children.end());
               }
               e->parent = orphan;
               e->from = orphan->to;
               orphan->children.push_back(e);
               e->cost = orphan->cost+orphan->to.dist(e->to);
               if (e->orphan && !seen.count((long)e)) {
                  Q.push(e); seen.insert((long)e);
               }
            }
         }
      }
   }

   inline void propagate_orphanhood(edge * self) {
      Q.push(self);
      while (!Q.empty()) {
         self = Q.front(); Q.pop();
         if (self->orphan)
            continue;
         self->orphan = true;
         points->remove(self);
         orphanage.add(self);
         self->parent = NULL;
         self->cost = DBL_MAX;
         for (edge * w : self->children) {
            Q.push(w);
         }
         self->children.clear();
      }
   }

};

#endif
