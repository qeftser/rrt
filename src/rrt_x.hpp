
#ifndef __RRT_X

#define __RRT_X
#include "weighted_edge.hpp"
#include "vertex.hpp"
#include "environment.hpp"
#include "rrt_base.hpp"
#include <queue>
#include <vector>
#include <cfloat>


class rrt_x : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   rrt_x() : env(NULL), points(NULL), ce(NULL) {}
   rrt_x(const vertex init, environment * env, point_set * p, collision_engine * c) : env(env), points(p), ce(c) {
      weighted_edge * start = new weighted_edge(init,init);
      points->start(start);
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         weighted_edge * nearest = (weighted_edge *)points->closest(random);

         /* state change computation
          * this is where we can extend to arbitrary DOF
          */
         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;

         /* get near vertices */
         std::vector<weighted_edge *> near;
         near.push_back(nearest);
         points->in_range(new_state,1.5,(std::vector<edge *> *)&near);

         /* find lowest cost for parent */
         weighted_edge * low_edge = get_parent(new_state,&near);

         /* insert new edge */
         if (low_edge && !low_edge->orphan) {
            weighted_edge * new_edge = new weighted_edge(low_edge->to,new_state,low_edge);
            new_edge->cost = low_edge->cost+low_edge->to.dist(new_state);
            low_edge->children.push_back(new_edge);
            points->add(new_edge);

            /* find any children to rewire */
            rewire_neighbors(new_edge,&near);
         }
      }
   }

   void notify_obstacle(const vertex pos) {
      std::vector<weighted_edge *> near;
      points->in_range(pos,1,(std::vector<edge *> *)&near);
      for (weighted_edge * e : near) {
         if (ce->is_collision(e->from,e->to)) {
            propagate_orphanhood(e);
         }
      }
restart:
      for (weighted_edge * e : near) {
         if (e->orphan) {
            points->remove(e);
            auto iterator = remove(near.begin(),near.end(),e);
            near.erase(iterator,near.end());
            delete e;
            goto restart;
         }
      }
      for (weighted_edge * e : near) {
         std::vector<weighted_edge *> local_near;
         points->in_range(e->to,1.5,(std::vector<edge *> *)&local_near);
         rewire_neighbors(e,&local_near);
      }
   }

private:

   inline weighted_edge * get_parent(const vertex & v, std::vector<weighted_edge *> * near) {
      double low_cost = DBL_MAX;
      weighted_edge * low_edge = NULL;
      for (weighted_edge * e : *near) {
         if (!e->orphan &&
             !ce->is_collision(e->to,v) &&
             e->cost+e->to.dist(v) < low_cost) {
            low_cost = e->cost+e->to.dist(v);
            low_edge = e;
         }
      }

      return low_edge;
   }

   inline void rewire_neighbors(weighted_edge * self, std::vector<weighted_edge *> * near) {
      for (weighted_edge * e : *near) {
         if (!ce->is_collision(self->to,e->to) &&
             self->cost+self->to.dist(e->to) < e->cost) {
            if (e->parent) {
               auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
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

   inline void adopt(weighted_edge * orphan) {
      std::vector<weighted_edge *> near;
      orphan->orphan = false;
      points->in_range(orphan->to,1.5,(std::vector<edge *> *)&near);
      rewire_neighbors(orphan,&near);
   }

   inline void propagate_orphanhood(weighted_edge * self) {
      /*
      if (self->parent) {
         auto result = remove(self->parent->children.begin(),self->parent->children.end(),self);
         self->parent->children.erase(result,self->parent->children.end());
      }
      */
      self->orphan = true;
      self->parent = NULL;
      self->cost = DBL_MAX;
      for (weighted_edge * w : *((std::vector<weighted_edge *> *)&self->children)) {
         propagate_orphanhood(w);
      }
      self->children.clear();
   }

};

#endif
