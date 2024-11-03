
#ifndef __RRT_X

#define __RRT_X
#include "rrt_x_edge.hpp"
#include "vertex.hpp"
#include "environment.hpp"
#include "rrt_base.hpp"
#include <queue>
#include <vector>

struct rrt_x_edge_compare {
   bool operator()(const rrt_x_edge * e1, const rrt_x_edge * e2) const {
      if ((e1->g < e1->lmc ? e1->g : e1->lmc) < (e2->g < e2->lmc ? e2->g : e2->lmc))
         return true;
      return false;
   }
};

class rrt_x : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   struct rrt_x_edge_compare rrt_x_edge_cmp;
   double E;
   double R;
   std::vector<rrt_x_edge *> Q;

   rrt_x() : env(NULL), points(NULL), ce(NULL) {}
   rrt_x(const vertex init, environment * env, point_set * p, collision_engine * c) : env(env), points(p), ce(c) {
      rrt_x_edge * start = new rrt_x_edge(init,init);
      points->start(start);
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         rrt_x_edge * nearest = (rrt_x_edge *)points->closest(random);

         /* state change computation
          * this is where we can extend to arbitrary DOF
          */
         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;

         

      }
   }

private:

   inline void find_parent(vertex new_state, 

   inline void extend(rrt_x_edge * v) {
      std::vector<rrt_x_edge *> near;
      points->in_range(v->to,R,(std::vector<edge *> *)&near);
      for (rrt_x_edge * u : near) {
         if (!ce->is_collision(u->to,v->to)) {
            if (!u->nr_in.count(v)) u->nr_in.insert(v);
            v->n0_in.insert(u);
            if (!u->nr_out.count(v)) u->nr_out.insert(v);
            v->n0_out.insert(u);
         }
      }
   }

   inline void reduce_inconsistancy() {
      while (!Q.empty()) {
         std::pop_heap(Q.begin(),Q.end());
         rrt_x_edge * v = Q.back(); Q.pop_back();
         if (v->g - v->lmc > E) {
            update_lmc(v);
            rewire_neighbors(v);
         }
         v->g = v->lmc;
      }
   }

   inline void update_lmc(rrt_x_edge * v) {
      cull_neighbors(v);
      rrt_x_edge * p = NULL;
      double best = v->lmc;
      for (rrt_x_edge * u : v->n0_out) {
         if (best > v->to.dist(u->to) + u->lmc) {
            p = u;
            best = v->to.dist(u->to) + u->lmc;
         }
      }
      for (rrt_x_edge * u : v->nr_out) {
         if (best > v->to.dist(u->to) + u->lmc) {
            p = u;
            best = v->to.dist(u->to) + u->lmc;
         }
      }
      if (p) {
         auto result = remove(v->parent->children.begin(),v->parent->children.end(),v);
         v->parent = p;
         p->children.push_back(v);
      }
   }
   
   inline void rewire_neighbors(rrt_x_edge * v) {
      if (v->g - v->lmc > E) {
         cull_neighbors(v);
         for (rrt_x_edge * u : v->n0_in) {
            if (u->lmc > u->to.dist(v->to) + v->lmc) {
               u->lmc = u->to.dist(v->to) + v->lmc;
               auto result = remove(u->parent->children.begin(),u->parent->children.end(),u);
               u->parent = v;
               v->children.push_back(u);
               if (u->g - u->lmc > E)
                  verrify_queue(u);
            }
         }
         for (rrt_x_edge * u : v->nr_in) {
            if (u->lmc > u->to.dist(v->to) + v->lmc) {
               u->lmc = u->to.dist(v->to) + v->lmc;
               auto result = remove(u->parent->children.begin(),u->parent->children.end(),u);
               u->parent = v;
               v->children.push_back(u);
               if (u->g - u->lmc > E)
                  verrify_queue(u);
            }
         }
      }
   }

   inline void verrify_queue(rrt_x_edge * v) {
      if (find(Q.begin(),Q.end(),v) != Q.end())
         make_heap(Q.begin(),Q.end(),rrt_x_edge_cmp);
      else {
         Q.push_back(v);
         push_heap(Q.begin(),Q.end());
      }
   }
   
   inline void cull_neighbors(rrt_x_edge * v) {
      // we are going to ignore this for now
   }
};

#endif
