#include "random_processing.hpp"

int rnd(int rnd1) // random in range [0; rng1)
{
   struct timespec time_n;
   clock_gettime(CLOCK_MONOTONIC, &time_n); //дает время с точностью до наносекунд
   int rnd2 = time_n.tv_nsec;
   srand(rnd2);
   return rnd1 != 0 ? (int)(rand() % rnd1) : 0;
} // -- END rnd()

int rnd(int min, int max) // [min; max]
{
   return min + rnd(max - min + 1); 
} // -- END rnd()

float rnd(float min, float max)
{
   struct timespec time_n;
   clock_gettime(CLOCK_MONOTONIC, &time_n); //дает время с точностью до наносекунд
   int rnd2 = time_n.tv_nsec;
   srand(rnd2);
   if (max - min !=0)
      return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
   else
      return min;
}

