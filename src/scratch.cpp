
#include "fibonacci_heap.hpp"
#include "binary_heap.hpp"
#include <queue>
#include <ctime>

void print_kv(int key, int val) {
   printf("(%d=%d)",key,val);
}

int main(void) {

   fibonacci_heap<int,int,std::less<int>> F;
   std::priority_queue<int> Q;
   binary_heap<int> B;

   srand(time(NULL)*clock());

   int total = 0xf;

   /*
   {
      clock_t tStart = clock();
      for (int i = 0; i < total; ++i) {
         int val = rand();
         F.insert(val,val);
      }
      printf("fibonacci load: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }
   {
      clock_t tStart = clock();
      for (int i = 0; i < total; ++i) {
         int val = rand();
         Q.push(val);
      }
      printf("priority_queue load: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }
   {
      clock_t tStart = clock();
      for (int i = 0; i < total; ++i) {
         int val = rand()%20;
         B.insert(val,val);
      }
      printf("binary load: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
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
   {
      clock_t tStart = clock();
      while (!B.empty()) {
         B.extract();
      }
      printf("binary dump: %f\n",(double)(clock()-tStart)/CLOCKS_PER_SEC);
   }
   */

   B.insert(1,1);
   B.insert(2,2);
   B.insert(5,5);
   B.insert(4,4);
   B.insert(7,7);
   B.insert(9,9);
   B.insert(3,3);

   B.print();

   B.update_key(1,10);
   
   int res = B.extract();
   printf("res: %d\n",res);

   return 0;
}
