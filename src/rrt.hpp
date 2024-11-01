
#ifndef __RRT

#define __RRT
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"
#include "collision_engine.hpp"
#include "rrt_base.hpp"

class rrt : public rrt_base {
public:
   environment * env;
   point_set * points;
   collision_engine * ce;

   rrt() : env(NULL), points(NULL), ce(NULL) {}
   rrt(const vertex init, environment * env, point_set * p, collision_engine * c) : env(env), points(p), ce(c) {
      edge start = edge(init,init);
      points->start(start);
   }

   void generate_next(int num) {
      ce->recalibrate();
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         edge * nearest = points->closest(random);
         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;
         if (!ce->is_collision(nearest->to,new_state)) {
            edge * new_edge = new edge(nearest->to,new_state,nearest);
            nearest->children.push_back(new_edge);
            points->add(new_edge);
         }
      }
   }

};

#endif
