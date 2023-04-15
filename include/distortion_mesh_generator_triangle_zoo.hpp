#ifndef DISTORTION_MESH_GENERATOR_TRIANGLE_HPP
#define DISTORTION_MESH_GENERATOR_TRIANGLE_HPP

#include<opencv2/core/types.hpp>

namespace meshGenerateTriangle
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createRandomWarpMes(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createDistortionWarpMesh(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
}

#endif //DISTORTION_MESH_GENERATOR_TRIANGLE_HPP
