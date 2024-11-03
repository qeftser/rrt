
#include "fibonacci_heap.hpp"
#include <queue>
#include <ctime>

void print_kv(int key, int val) {
   printf("(%d=%d)",key,val);
}

int main(void) {

   fibonacci_heap<int,int,std::less<int>> F;
   std::priority_queue<int> Q;

   srand(time(NULL)*clock());

   {
      clock_t tStart = clock();
      for (int i = 0; i < 0x1ffffff; ++i) {
         int val = rand();
         F.insert(val,val);
      }
      printf("fibonacci load: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }
   {
      clock_t tStart = clock();
      for (int i = 0; i < 0x1ffffff; ++i) {
         int val = rand();
         Q.push(val);
      }
      printf("priority_queue load: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }


   {
      clock_t tStart = clock();
      while (!F.empty()) {
         F.extract_min();
      }
      printf("fibonacci dump: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }
   
   {
      clock_t tStart = clock();
      while (!Q.empty()) {
         Q.pop();
      }
      printf("priority_queue dump: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }

   return 0;
}
