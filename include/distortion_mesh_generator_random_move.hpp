#ifndef DISTORTION_MESH_GENERATOR_RANDOM_MOVE
#define DISTORTION_MESH_GENERATOR_RANDOM_MOVE

#include <opencv2/core/types.hpp>

namespace mesh_generator::random_move
{
    cv::Mat createRandomWarpMesh(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize, 
        cv::Size meshGridSize);
}


#endif //DISTORTION_MESH_GENERATOR_RANDOM_MOVE