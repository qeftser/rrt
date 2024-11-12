
#ifndef __RRT_X_FN

#define __RRT_X_FN
#include "vertex.hpp"
#include "environment.hpp"
#include "rrt_base.hpp"
#include "bin_point_set.hpp"
#include "collision_engine.hpp"
#include <queue>
#include <vector>
#include <cfloat>
#include <unordered_set>

class rrt_x_fn : public rrt_base {
public:

   const double R = 1.0;
   const double cull_range = 4.5;
   const int fixed_limit = 25000;
   int count = 0;

   environment * env;
   point_set * points;
   collision_engine * ce;
   bin_point_set orphanage;

   std::queue<edge *> Q;
   std::unordered_set<long> infertile;

   rrt_x_fn() : env(NULL), points(NULL), ce(NULL),
                orphanage(bin_point_set(5)) {}
   rrt_x_fn(const vertex init, environment * env, point_set * p, collision_engine * c)
      : env(env), points(p), ce(c), orphanage(bin_point_set(5)) {
      edge * start = new edge(init,init);
      points->add(start);
      infertile.insert((long)start);
   }

   ~rrt_x_fn() {
      orphanage.reset();
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         edge * nearest = points->closest(random);

         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;

         std::vector<edge *> near;
         near.push_back(nearest);
         points->in_range(new_state,R,&near);

         edge * low_edge = get_parent(new_state,&near);

         if (low_edge) {
            edge * new_edge = new edge(low_edge->to,new_state,low_edge);
            new_edge->cost = low_edge->cost+low_edge->to.dist(new_state);
            if (low_edge->children.empty())
               infertile.erase((long)low_edge);
            low_edge->children.push_back(new_edge);
            points->add(new_edge);
            infertile.insert((long)new_edge);
            ++count;

            orphanage.in_range(new_state,R,&near);
            rewire_neighbors(new_edge,&near);
         }

         if (count > fixed_limit) {
            edge * sacrifice = random_sacrifice();
            infertile.erase((long)sacrifice);
            --count;
            if (sacrifice->parent) {
               auto result = remove(sacrifice->parent->children.begin(),sacrifice->parent->children.end(),sacrifice);
               sacrifice->parent->children.erase(result,sacrifice->parent->children.end());
               points->remove(sacrifice);

               if (sacrifice->parent->children.empty())
                  infertile.insert((long)sacrifice->parent);
            }            
            else
               orphanage.remove(sacrifice);
            delete sacrifice;
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
               if (e->parent->children.empty())
                  infertile.insert((long)e->parent);
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
            infertile.erase((long)e);
            delete e;
            goto restart;
         }
      }
   }

   void restart(const vertex pos) {
      points->reset();
      orphanage.reset();
      edge * start = new edge(pos,pos);
      points->add(start);
      infertile.clear();
      infertile.insert((long)start);
      count = 1;
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
reset:
      for (edge * e : *near) {
         bool reset_needed = false;
         if (!ce->is_collision(self->to,e->to) &&
             self->cost+self->to.dist(e->to) < e->cost) {
            if (e->parent) {
               auto result = remove(e->parent->children.begin(),e->parent->children.end(),e);
               e->parent->children.erase(result,e->parent->children.end());
               if (e->parent->children.empty()) {
                  auto result2 = remove(e->parent->parent->children.begin(),
                                        e->parent->parent->children.end(),
                                        e->parent);
                  e->parent->parent->children.erase(result2,
                        e->parent->parent->children.end());
                  points->remove(e->parent);
                  auto result3 = remove(near->begin(),near->end(),e->parent);
                  near->erase(result3,near->end());
                  delete e->parent;
                  --count;
                  reset_needed = true;
               }
            }
            e->parent = self;
            e->from = self->to;
            if (self->children.empty())
               infertile.erase((long)self);
            self->children.push_back(e);
            e->cost = self->cost+self->to.dist(e->to);
            if (e->orphan)
               adpot(e);
            if (reset_needed)
               goto reset;
         }
      }
   }

   inline void adpot(edge * orphan) {
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
                  if (e->parent->children.empty())
                     infertile.insert((long)e->parent);
               }
               e->parent = orphan;
               e->from = orphan->to;
               if (orphan->children.empty())
                  infertile.erase((long)orphan);
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
      while(!Q.empty()) {
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
         infertile.insert((long)self);
      }
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
