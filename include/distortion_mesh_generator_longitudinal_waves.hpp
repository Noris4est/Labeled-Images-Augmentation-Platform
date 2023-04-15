#ifndef DISTORTION_MESH_GENERATOR_LONGITUDINAL_WAVES_HPP
#define DISTORTION_MESH_GENERATOR_LONGITUDINAL_WAVES_HPP

#include <opencv2/opencv.hpp>
#include "mesh_grid_nodes_mover_common.hpp"
#include <functional>


//Longitudinal waves 
namespace mesh_generator::longitudinal
{
    void createLongitudinalWaveSinWarpMeshDirectionX(const cv::Mat &primeMesh_src, cv::Mat &warpMesh_dst, cv::Size frameSize_src, cv::Size meshGridSize_src, cv::Size callbackMeshSize_src);
    void createLongitudinalWaveSinWarpMeshDirectionY(const cv::Mat &primeMesh_src, cv::Mat &warpMesh_dst, cv::Size frameSize_src, cv::Size meshGridSize_src, cv::Size callbackMeshSize_src);

    void createLongitudinalWaveBaseCallbackWarpMesh(
        std::function<double(double)> base_callback,
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Size callbackMeshSize_src,
        mesh_nodes_move::WaveCallbackMeshPropagationAxis axis,
        bool change_position_left_and_right_border_mesh_nodes = false,
        bool change_position_bottom_and_top_border_mesh_nodes = false);    

    void createLongitudinalWaveSinWarpMeshDistortion(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Size callbackMeshSize_src,
        mesh_nodes_move::WaveCallbackMeshPropagationAxis axis,
        bool change_position_left_and_right_border_mesh_nodes = false,
        bool change_position_bottom_and_top_border_mesh_nodes = false);

    void createLongitudinalWaveGammaWarpMeshDistortion(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Size callbackMeshSize_src,
        double gamma_coefficient,
        mesh_nodes_move::WaveCallbackMeshPropagationAxis axis,
        bool change_position_left_and_right_border_mesh_nodes = false,
        bool change_position_bottom_and_top_border_mesh_nodes = false);

    // Генерация концентричных волн из некоторой точки истока(стока)
    void createLongitudinalWaveBaseCallbackFromSourcePointConcentric(
        std::function<double(double)> base_callback,
        const cv::Mat &primeMesh_src,
        cv::Mat &warpMesh_dst,
        cv::Size frameSize_src,
        cv::Size meshGridSize_src,
        cv::Point sourcePoint,
        double halfPeriodOfWaveDividedByMeshCellDiag);

    // Концентричные продольные синусоидальные волны, исходящие из некоторой точки - истока (или стока)
    void createLongitudinalWaveSinFromSourcePointConcentric(
        const cv::Mat &primeMesh_src,
        cv::Mat &warpMesh_dst,
        cv::Size frameSize_src,
        cv::Size meshGridSize_src,
        cv::Point sourcePoint,
        double halfPeriodOfWaveDividedByMeshCellDiag); // коэффициент, во сколько раз период расходящейся продольной волны больше 
}

#endif // DISTORTION_MESH_GENERATOR_LONGITUDINAL_WAVES_HPP