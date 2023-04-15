#include "math_utils_common.hpp"

float ipow(float x, int n)
{
    float product = 1;
    for(int i = 0; i < n; ++i)
    {
        product *= x;
    }
    return product;
}
