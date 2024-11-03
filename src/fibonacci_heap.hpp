
#ifndef __FIBONACCI_HEAP

#define __FIBONACCI_HEAP
#include <cstdlib>
#include <set>
#include <cstdio>
#include <cmath>
#include <cstring>

template <typename K, typename V, class Cmp>
class fibonacci_heap {
private:
   
   struct fib_node {
      struct fib_node * prev;
      struct fib_node * next;
      struct fib_node * parent;
      struct fib_node * child;
      short mark;
      short degree;
      K key;
      V val;
   };

   struct fib_node * min;
   int num;
   const double log_golden_ratio = log((1.0+sqrt(5.0))/2.0);

   Cmp compare;

   void consolidate();
   void print_aux(void (*) (K, V), struct fib_node *, std::set<long> *);

   inline void fib_heap_link(struct fib_node * y, struct fib_node * x) {
      if (y->next != y) {
         y->next->prev = y->prev;
         y->prev->next = y->next;
      }
      y->next = y;
      y->prev = y;
      y->mark = 0;
      y->parent = x;
      if (!x->child)
         x->child = y;
      else {
         y->next = x->child->next;
         y->prev = x->child;
         x->child->next->prev = y;
         x->child->next = y;
         if (compare(y->key,x->child->key))
            x->child = y;
      }
      x->degree = x->degree + 1;
   }

public: 

   fibonacci_heap() : min(NULL), num(0) { compare = Cmp();}
   fibonacci_heap(fibonacci_heap<K,V,Cmp> & H1, fibonacci_heap<K,V,Cmp> & H2) {
      compare = Cmp();
      if (!H1.min) {
         if (!H2.min) {
            min = NULL;
            num = 0;
            return;
         }
         min = H2.min;
         return;
      }
      else {
         if (!H2.min) {
            min = H1.min;
            return;
         }
         else {
            if (!compare(H1.min->key,H2.min->key))
               min = H1.min;
            else
               min = H2.min;
         }
      }
      struct fib_node * temp = H1.min->next;
      H1.min->next = H2.min->next;
      H2.min->next->prev = H1.min;
      H2.min->next = temp;
      temp->prev = H2.min;
      num = H1.num + H2.num;
   }

   bool empty();
   void insert(K, V);
   V minimum();
   V extract_min();
   void decrease_key(V, K);
   void remove(V);

   void print(void (*) (K,V));
static void print_kv(int key, int val) {
   printf("(%d=%d)",key,val);
}
};

template <typename K, typename V, class Cmp>
bool fibonacci_heap<K,V,Cmp>::empty() {
   return (0 == num);
}

template <typename K, typename V, class Cmp>
V fibonacci_heap<K,V,Cmp>::minimum() {
   return min->val;
}

template <typename K, typename V, class Cmp>
void fibonacci_heap<K,V,Cmp>::insert(K k, V v) {
   struct fib_node * x_node = (struct fib_node *)malloc(sizeof(struct fib_node));
   x_node->degree = 0;
   x_node->parent = NULL;
   x_node->child = NULL;
   x_node->mark = 0;
   x_node->val = v;
   x_node->key = k;
   if (!min) {
      x_node->prev = x_node;
      x_node->next = x_node;
      min = x_node;
   }
   else {
      x_node->next = min->next;
      min->next->prev = x_node;
      x_node->prev = min;
      min->next = x_node;
      if (compare(k,min->key))
         min = x_node;
   }
   ++num;
}

template <typename K, typename V, class Cmp>
V fibonacci_heap<K,V,Cmp>::extract_min() {
   struct fib_node * z = min;
   if (z->child) {
      struct fib_node * x, * temp, * curr = z->child;
      x = curr;
      do {
         temp = curr->next;

         curr->next = z->next;
         z->next->prev = curr;
         curr->prev = z;
         z->next = curr;
         curr->parent = NULL;

         curr = temp;
      } while (x != curr);
   }
   z->next->prev = z->prev;
   z->prev->next = z->next;
   if (z == z->next)
      min = NULL;
   else {
      min = z->next;
      consolidate();
   }
   V val = z->val;
   free(z);
   --num;
   return val;
}

template <typename K, typename V, class Cmp>
void fibonacci_heap<K,V,Cmp>::consolidate() {
   int DHn = ceil(log(num)/log_golden_ratio);
   struct fib_node * A[DHn];
   bzero(A,DHn*sizeof(struct fib_node *));

   struct fib_node * temp, * curr, * start = min;
   curr = start->next;
   do {
      struct fib_node * x = curr;
      temp = curr->next;

      int d = x->degree;
      while (A[d]) {
         struct fib_node * y = A[d];
         if (!compare(x->key,y->key)) {
            A[d] = x;
            x = y;
            y = A[d];
         }
         fib_heap_link(y,x);
         A[d] = NULL;
         d = d + 1;
      }
      A[d] = x;
   } while (curr != start && (curr = temp));
   min = NULL;
   for (int i = 0; i < DHn; ++i) {
      if (A[i] != NULL) {
         if (!min) {
            min = A[i];
            min->next = min;
            min->prev = min;
         }
         else {
            A[i]->next = min->next;
            min->next->prev = A[i];
            A[i]->prev = min;
            min->next = A[i];
            if (compare(A[i]->key,min->key))
               min = A[i];
         }
      }
   }
}

template <typename K, typename V, class Cmp>
void fibonacci_heap<K,V,Cmp>::print(void (* print_func) (K,V)) {
   auto set = new std::set<long>();
   print_aux(print_func,min,set);
   printf("\n");
   delete set;
}

template <typename K, typename V, class Cmp>
void fibonacci_heap<K,V,Cmp>::print_aux(void (* print_func) (K,V), struct fib_node * curr, std::set<long> * seen) {
   if (!curr || seen->count((long)curr))
      return;
   printf("{");
   print_func(curr->key,curr->val);
   printf("}");
   printf("[");
   seen->insert((long)curr);
   print_aux(print_func,curr->child,seen);
   printf("]");
   print_aux(print_func,curr->next,seen);
}




#endif
