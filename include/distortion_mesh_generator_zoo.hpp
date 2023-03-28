#ifndef DISTORTION_MESH_GENERATOR_ZOO_HPP
#define DISTORTION_MESH_GENERATOR_ZOO_HPP

#include <opencv2/opencv.hpp>

namespace meshGenerator
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно плиточки (полигоны)
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createExpansionWarpMeshFromCenter2BordersSquare(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createExpansionWarpMeshFromCenter2BordersCircle(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createRandomWarpMes(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createDistortionWarpMesh(cv::Mat &mesh, cv::Size framesize, cv::Size meshgridsize);
}

namespace meshGenerateTriangle
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createRandomWarpMes(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createDistortionWarpMesh(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
}
#endif //DISTORTION_MESH_GENERATOR_ZOO_HPP