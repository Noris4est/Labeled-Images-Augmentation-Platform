#ifndef DISTORTION_MESH_GENERATOR_RANDOM_MOVE
#define DISTORTION_MESH_GENERATOR_RANDOM_MOVE

#include <opencv2/core/types.hpp>

namespace mesh_generator::random_move
{

    void createRandomWarpMesh(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize, 
        cv::Size meshGridSize, 
        bool random_relative_amplitude = false, 
        double rel_amplitude = 0.5,
        std::pair<float, float> random_amplitude_range = {0.1f,0.3f});
}

#endif //DISTORTION_MESH_GENERATOR_RANDOM_MOVE