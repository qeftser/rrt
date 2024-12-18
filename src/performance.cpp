
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>

#include "environment.hpp"

#include "point_set.hpp"
#include "simple_point_set.hpp"
#include "bin_point_set.hpp"
#include "hyperbin_point_set.hpp"

#include "collision_engine.hpp"
#include "dda_collision_engine.hpp"

#include "rrt_base.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "quick_rrt_star.hpp"
#include "rrt_x_fn.hpp"

int main(void) {

   int count = 10000;

   {
      vertex start = vertex(100,100);
      environment env = environment(200,200);


      printf("USING SIMPLE POINT SET\n");
      printf("Computing %d edges in an empty environment\n",count);
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 1000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a sparse environment\n",count);
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 5000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a dense environment\n",count);
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new simple_point_set();
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int divisor = 100; divisor > 0; divisor /= 10) {
         env.clear();
      printf("\nUSING BIN POINT SET WITH DIVISOR %d\n",divisor);
      printf("Computing %d edges in an empty environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 1000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a sparse environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 5000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a dense environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         quick_rrt_star r = quick_rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","Q-RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      }

      count = 100000;

      /*
      for (int divisor = 10; divisor > 0; --divisor) {
         env.clear();
      printf("\nUSING BIN POINT SET WITH DIVISOR %d\n",divisor);
      printf("Computing %d edges in an empty environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 1000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a sparse environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 5000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a dense environment\n",count);
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new bin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      }
      */

      for (int divisor = 10; divisor > 0; --divisor) {
         env.clear();
      printf("\nUSING HYPERBIN POINT SET WITH DIVISOR %d\n",divisor);
      printf("Computing %d edges in an empty environment\n",count);
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }

      for (int i = 0; i < 1000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a sparse environment\n",count);
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      
      for (int i = 0; i < 5000; ++i)
         env.set(rand()%200,rand()%200);

      printf("Computing %d edges in a dense environment\n",count);
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt r = rrt(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_star r = rrt_star(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRT*");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      {
         point_set * points = new hyperbin_point_set(divisor);
         collision_engine * ce = new dda_collision_engine(&env);
         rrt_x_fn r = rrt_x_fn(start,&env,points,ce);

         fprintf(stderr,"%10s\t:\t","RRTXFN");
         clock_t tStart = clock();
         r.generate_next(count);
         printf("%f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
      }
      }

   }

   return 0;
}
