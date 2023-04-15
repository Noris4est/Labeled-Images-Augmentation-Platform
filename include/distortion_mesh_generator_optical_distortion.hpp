#ifndef DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION
#define DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION

#include <opencv2/core/types.hpp>

namespace mesh_generator::optical_distortion
{
    void createDistortionWarpMesh(
        const cv::Mat primeMesh_src, 
        cv::Mat &warpMesh_dst,
        cv::Size frameSize, 
        cv::Size meshGridSize); // оптическая дисторсия]
} // namespace mesh_generator::optical_distortion


#endif //DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION