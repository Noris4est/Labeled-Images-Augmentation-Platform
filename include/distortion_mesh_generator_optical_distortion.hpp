#ifndef DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION
#define DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION

#include <opencv2/core/types.hpp>

namespace mesh_generator::optical_distortion
{
    struct OpticalDistortionSettings
    {
        double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0; // основной - k1 (при k1 < 0 - бочка, k1 > 0 - подушка)
    };

    void createDistortionWarpMesh(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst,
        cv::Size frameSize, 
        cv::Size meshGridSize,
        OpticalDistortionSettings settings); // оптическая дисторсия]
} // namespace mesh_generator::optical_distortion


#endif //DISTORTION_MESH_GENERATOR_OPTICAL_DISTORTION