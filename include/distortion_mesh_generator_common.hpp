#ifndef DISTORTION_MESH_GENERATOR_COMMON_HPP
#define DISTORTION_MESH_GENERATOR_COMMON_HPP

#include <opencv2/core/types.hpp>

namespace mesh_generator::utils
{
    void checkValidSrcMeshGrid(const cv::Mat &src_mesh);
}

#endif // DISTORTION_MESH_GENERATOR_COMMON_HPP