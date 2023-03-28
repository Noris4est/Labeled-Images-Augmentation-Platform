#ifndef RANDOM_PROCESSING
#define RANDOM_PROCESSING

#include <ctime>
#include <cstdlib>

int rnd(int rnd1); // random in range [0; rng1)
int rnd(int min, int max); // [min; max]
float rnd(float min, float max); // []
#endif //RANDOM_PROCESSING