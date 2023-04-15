#ifndef DISTORTION_MESH_GENERATOR_ZOO_HPP
#define DISTORTION_MESH_GENERATOR_ZOO_HPP

#include <opencv2/opencv.hpp>
#include "mesh_grid_nodes_mover_common.hpp"
#include "distortion_mesh_generator_longitudinal_waves.hpp"
#include "distortion_mesh_generator_transverse_waves.hpp"
#include "distortion_mesh_generator_optical_distortion.hpp"
#include "distortion_mesh_generator_random_move.hpp"
#include "distortion_mesh_generator_triangle_zoo.hpp"

namespace mesh_generator
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно плиточки (полигоны)

    namespace various_others
    {
        cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
        cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
        cv::Mat createExpansionWarpMeshFromCenter2BordersSquare(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
        cv::Mat createExpansionWarpMeshFromCenter2BordersCircle(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    }

}

#endif //DISTORTION_MESH_GENERATOR_ZOO_HPP