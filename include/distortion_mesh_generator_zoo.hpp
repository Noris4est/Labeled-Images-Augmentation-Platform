#ifndef DISTORTION_MESH_GENERATOR_ZOO_HPP
#define DISTORTION_MESH_GENERATOR_ZOO_HPP

#include <opencv2/opencv.hpp>
#include "mesh_grid_nodes_mover_common.hpp"

namespace meshGenerator
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно плиточки (полигоны)
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createExpansionWarpMeshFromCenter2BordersSquare(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createExpansionWarpMeshFromCenter2BordersCircle(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat createRandomWarpMes(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize);
    
    
    cv::Mat createDistortionWarpMesh(cv::Mat &mesh, cv::Size framesize, cv::Size meshgridsize); // оптическая дисторсия

    //transverse waves 

    //TODO: Сделать поперечный синус под углом к линии горизонта - задается как параметр angle

    cv::Mat createTransverseSinWarpMesh_propX_changeY(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor = 0.2);
    cv::Mat createTransverseSinWarpMesh_propY_changeX(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor = 0.2);

    //scalefactor_X - масштабный коэффициент смешения вершин сетки по оси x
    cv::Mat createTransverseSinWarpMesh_propXY_changeXY(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor_X = 0.3, float scaleFactor_Y = 0.1);
    

    //Longitudinal waves 
    namespace longitudinal
    {

    }
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

namespace meshGenerateTriangle
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createRandomWarpMes(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize); // meshGridSize - именно полигоны
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
    cv::Mat createDistortionWarpMesh(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize);
}
#endif //DISTORTION_MESH_GENERATOR_ZOO_HPP