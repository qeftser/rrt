
#ifndef __RRT

#define __RRT
#include "environment.hpp"
#include "edge.hpp"
#include "vertex.hpp"
#include "point_set.hpp"

class rrt {
public:
   environment * env;
   point_set * points;

   rrt() : env(NULL), points(NULL) {}
   rrt(const vertex init, environment * env, point_set * p) : env(env), points(p) {
      edge * start = new edge(init,init);
      points->add(start);
   }

   void generate_next(int num) {
      for (int i = 0; i < num; ++i) {
         vertex random = vertex::rand(env->xsize,env->ysize);
         edge * nearest = points->closest(random);
         vertex new_state = (random-nearest->to);
         new_state = (new_state/new_state.dist(vertex()))+nearest->to;
         if (!0) {//edge::is_collision(nearest->to,new_state,env)) {
            edge * new_edge = new edge(nearest->to,new_state,nearest);
            nearest->children.push_back(new_edge);
            points->add(new_edge);
         }
      }
   }

};

#endif
