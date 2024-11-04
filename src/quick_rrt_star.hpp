
#ifndef __QUICK_RRT_STAR

#define __QUICK_RRT_STAR
#include "environment.hpp"
#include "edge.hpp"
#include "weighted_edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "collision_engine.hpp"
#include "simple_collision_engine.hpp"
#include "rrt_base.hpp"
#include <cfloat>


class quick_rrt_star : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   quick_rrt_star() : env(NULL), points(NULL), ce(NULL) {}
   quick_rrt_star(const vertex init, environment * env, point_set * p, collision_engine * c) : env(env), points(p), ce(c) {
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
         points->in_range(new_state,1.1,(std::vector<edge *> *)&near);

         /* find lowest cost for parent */
         double low_cost = DBL_MAX;
         weighted_edge * low_edge = NULL;
         weighted_edge * ancestor;
         for (weighted_edge * e : near) {
            int depth = 5;
            ancestor = e;
            do {
               if (!ce->is_collision(ancestor->to,new_state) &&
                   ancestor->cost+ancestor->to.dist(new_state) < low_cost) {
                  low_cost = ancestor->cost+e->to.dist(new_state);
                  low_edge = ancestor;
               }
               ancestor = (weighted_edge *)ancestor->parent;
            } while (--depth && ancestor); 
         }

         /* insert new edge */
         if (low_edge) {
            weighted_edge * new_edge = new weighted_edge(low_edge->to,new_state,low_edge);
            new_edge->cost = low_cost;
            low_edge->children.push_back(new_edge);
            points->add(new_edge);

            /* find any children to rewire */
            int depth;
            weighted_edge * best, * curr;
            for (weighted_edge * e : near) {
               best = NULL;
               curr = new_edge;
               depth = 5;
               if (new_edge->cost+new_edge->to.dist(e->to) < e->cost) {
                  while (curr && depth-- &&
                        !ce->is_collision(curr->to,e->to)) {
                     best = curr;
                     curr = (weighted_edge *)curr->parent;
                  }
                  if (best) {
                     auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
                     e->parent = best;
                     e->from = best->to;
                     best->children.push_back(e);
                     e->cost = best->cost+best->to.dist(e->to);
                  }
               }
            }
         }
      }
   }


};

#endif
