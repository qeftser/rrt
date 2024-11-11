
#ifndef __RRT_STAR_FN

#define __RRT_STAR_FN
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "bin_point_set.hpp"
#include "collision_engine.hpp"
#include "rrt_base.hpp"
#include <cfloat>
#include <unordered_set>

class rrt_star_fn : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   std::unordered_set<long> infertile;
   
   const int fixed_limit = 25000;
   int count = 0;

   rrt_star_fn() : env(NULL), points(NULL), ce(NULL) {}
   rrt_star_fn(const vertex init, environment * env, point_set * p, collision_engine * c) 
      : env(env), points(p), ce(c) {
      edge * start = new edge(init,init);
      points->add(start);
      infertile.insert((long)start);
      ++count;
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
         points->in_range(new_state,1.5,(std::vector<edge *> *)&near);

         /* find lowest cost for parent */
         double low_cost = DBL_MAX;
         edge * low_edge = NULL;
         for (edge * e : near) {
            if (!ce->is_collision(e->to,new_state) &&
                e->cost+e->to.dist(new_state) < low_cost) {
               low_cost = e->cost+e->to.dist(new_state);
               low_edge = e;
            }
         }

         /* insert new edge */
         if (low_edge) {
            edge * new_edge = new edge(low_edge->to,new_state,low_edge);
            new_edge->cost = low_cost;
            low_edge->children.push_back(new_edge);
            if (infertile.count((long)low_edge))
               infertile.erase((long)low_edge);
            points->add(new_edge);
            infertile.insert((long)new_edge);
            ++count;

            /* find any children to rewire */
restart:
            for (edge * e : near) {
               bool reset_needed = false;
               if (!ce->is_collision(new_state,e->to) &&
                   new_edge->cost+new_state.dist(e->to) < e->cost) {
                  auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
                  e->parent->children.erase(result,e->parent->children.end());
                  if (e->parent->children.empty()) {
                     auto result2 = remove(e->parent->parent->children.begin(),
                                           e->parent->parent->children.end(),
                                           e->parent);
                     e->parent->parent->children.erase(result2,
                           e->parent->parent->children.end());
                     points->remove(e->parent);
                     auto result3 = remove(near.begin(),near.end(),e->parent);
                     near.erase(result3,near.end());
                     delete e->parent;
                     --count;
                     reset_needed = true;
                  }
                  e->parent = new_edge;
                  e->from = new_edge->to;
                  new_edge->children.push_back(e);
                  e->cost = new_edge->cost+new_state.dist(e->to);
                  if (reset_needed)
                     goto restart;
               }
            }

            if (!new_edge->children.empty())
               infertile.erase((long)new_edge);

            if (count > fixed_limit) {
               edge * sacrifice = random_sacrifice();
               infertile.erase((long)sacrifice);
               --count;
               auto result = remove(sacrifice->parent->children.begin(),sacrifice->parent->children.end(),sacrifice);
               sacrifice->parent->children.erase(result,sacrifice->parent->children.end());
               points->remove(sacrifice);
               if (sacrifice->parent->children.empty())
                  infertile.insert((long)sacrifice->parent);
               delete sacrifice;
            }

         }
      }
   }

   void restart(const vertex pos) {
      points->reset();
      edge * start = new edge(pos,pos);
      points->add(start);
      infertile.clear();
      infertile.insert((long)start);
      count = 0;
   }

private:

   inline edge * random_sacrifice() {
      int idx = rand()%infertile.size();
      auto it = infertile.begin();
      for (int i = 0; i < idx; ++i)
         ++it;
      return (edge *)*it;
   }
};


#endif
