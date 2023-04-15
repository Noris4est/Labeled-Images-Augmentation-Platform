#include "distortion_mesh_generator_common.hpp"

namespace mesh_generator::utils
{
    void checkValidSrcMeshGrid(const cv::Mat &src_mesh)
    {
        
    }
float ipow(float x, int n)
{
    float product = 1;
    for(int i = 0; i < n; ++i)
    {
        product *= x;
    }
    return product;
}

}
