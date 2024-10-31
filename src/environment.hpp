
#ifndef __ENVIROMENT

#define __ENVIROMENT
#include <cstdlib>

class environment {
public:
   int ** occupancy;
   int xsize;
   int ysize;

   environment() : xsize(0), ysize(0), occupancy(NULL) {}

   environment(int xsize, int ysize) {
      this->xsize = xsize;
      this->ysize = ysize;
      occupancy = (int **)malloc(sizeof(int *)*xsize);
      for (int i = 0; i < xsize; ++i)
         occupancy[i] = (int *)calloc(sizeof(int),ysize);
   }

   ~environment() {
      for (int i = 0; i < xsize; ++i)
         free(occupancy[i]);
      free(occupancy);
   }

   void set(int x, int y) {
      if (x < 0 || x >= xsize || y < 0 || y > ysize)
         return;
      occupancy[x][y] = 1;
   }

   void unset(int x, int y) {
      if (x < 0 || x >= xsize || y < 0 || y > ysize)
         return;
      occupancy[x][y] = 0;
   }
};

#endif
