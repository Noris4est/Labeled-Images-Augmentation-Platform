#ifndef DISTORTION_MESH_GENERATOR_TRANSVERSE_WAVES_HPP
#define DISTORTION_MESH_GENERATOR_TRANSVERSE_WAVES_HPP

#include <opencv2/core/types.hpp>
//transverse waves 

namespace mesh_generator::transverse
{
    //TODO: Сделать поперечный синус под углом к линии горизонта - задается как параметр angle
    //const cv::Mat &primeMesh_src, cv::Mat &warpMesh_dst, cv::Size frameSize_src, cv::Size meshGridSize_src, cv::Size callbackMeshSize_src
    void createTransverseSinWarpMesh_propX_changeY(
        const cv::Mat primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor = 0.2);
    void createTransverseSinWarpMesh_propY_changeX(
        const cv::Mat primeMesh_src,
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor = 0.2);

    //scalefactor_X - масштабный коэффициент смешения вершин сетки по оси x
    void createTransverseSinWarpMesh_propXY_changeXY(
        const cv::Mat primeMesh_src,
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor_X = 0.2,
        float scaleFactor_Y = 0.2);
};

#endif // DISTORTION_MESH_GENERATOR_TRANSVERSE_WAVES_HPP