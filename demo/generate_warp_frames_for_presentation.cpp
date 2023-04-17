#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <polygon_processing.hpp>
#include "warp_mesh_applicator.hpp"
#include "distortion_mesh_generator_zoo.hpp"
#include "utils.hpp"
#include "frame_warp_processing.hpp"
#include "draw_custom_line.hpp"
#include "colors.hpp"
#include "mesh_grid_nodes_mover_common.hpp"
#include "path_processing.hpp"
int main(int kargs, char* kwargs[])
{
    std::string results_save_dir = "../data/16_04_23/ex11";
    if(!path_processing::isDirExist(results_save_dir))
    {
        path_processing::makePath(results_save_dir);
    }
    
    cv::Size workFrameSize = {800, 800};
    cv::Size meshGridSize = {20,20};
    cv::Size callbackMeshGridSize_x_axis = {16,1};
    cv::Size callbackMeshGridSize_y_axis = {1,16};
    
    cv::Mat srcMesh, dstMesh, proxyMesh;
    // srcMesh = meshGenerator::creatSqrtWarpMeshFromLeft2Right(dstMesh, workFrameSize, cv::Size(14, 8));
    // srcMesh = meshGenerator::creatSqrtWarpMeshFromTopLeft2BottomRight(dstMesh, workFrameSize, cv::Size(7, 7));
    // srcMesh = meshGenerator::createExpansionWarpMeshFromCenter2BordersSquare(dstMesh, cv::Size(600,400), cv::Size(7, 5));
    // srcMesh = meshGenerator::createExpansionWarpMeshFromCenter2BordersCircle(dstMesh, cv::Size(500,500), cv::Size(5, 3));
//     srcMesh = meshGenerator::createRandomWarpMes(dstMesh, cv::Size(500,500), cv::Size(3, 3));
    // srcMesh = meshGenerator::createDistortionWarpMesh(dstMesh, workFrameSize, cv::Size(7, 7));
    // srcMesh = meshGenerator::createSinWarpMesh_propX_changeY(dstMesh, workFrameSize, cv::Size(25, 5));
    // srcMesh = meshGenerator::createSinWarpMesh_propY_changeX(dstMesh, workFrameSize, cv::Size(5, 25)); // TODO: разобраться, почему 50 width size падает с ошибкой
    // srcMesh = meshGenerator::createSinWarpMesh_propXY_changeXY(dstMesh, workFrameSize, cv::Size(12, 12), 0.3, 0.3);


    srcMesh = mesh_generator::createPrimeMesh(workFrameSize, meshGridSize);
    // meshGenerator::createWaveSinWarpMeshDirectionX(srcMesh, proxyMesh, workFrameSize, meshGridSize, callbackMeshGridSize);
    // meshGenerator::createWaveSinWarpMeshDirectionY(proxyMesh, dstMesh, workFrameSize, meshGridSize, callbackMeshGridSize_y_axis);
    // meshGenerator::createLongitudinalWaveGammaWarpMeshDistortion(
    //     srcMesh, 
    //     dstMesh, 
    //     workFrameSize, 
    //     meshGridSize, 
    //     callbackMeshGridSize_x_axis, 2, mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX, false, true);
    
    // meshGenerator::createLongitudinalWaveGammaWarpMeshDistortion(
    //     dstMesh, 
    //     dstMesh, 
    //     workFrameSize, 
    //     meshGridSize, 
    //     callbackMeshGridSize_y_axis, 2, mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisY, true, false);
   
    // meshGenerator::createLongitudinalWaveSinWarpMeshDistortion(
    //     srcMesh,
    //     dstMesh,
    //     workFrameSize,
    //     meshGridSize,
    //     callbackMeshGridSize_x_axis,
    //     mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
    //     false,
    //     true);
    double gamma_coeff = 1.6;
    cv::Point source_wave_point = {150,150};
    double halfPeriodOfWaveDividedByMeshCellDiag = 2;
    // mesh_generator::longitudinal::concentric_spherical_waves::createLongitudinalWaveGammaFromSourcePointConcentric(
    //     srcMesh,
    //     dstMesh,
    //     workFrameSize,
    //     meshGridSize,
    //     source_wave_point,
    //     gamma_coeff,
    //     halfPeriodOfWaveDividedByMeshCellDiag);

    // mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveSinWarpMesh(
    //     srcMesh,
    //     dstMesh,
    //     workFrameSize,
    //     meshGridSize,
    //     1.5,
    //     mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
    //     -M_PI/3,
    //     true);

    // mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveSinWarpMesh(
    //     dstMesh,
    //     dstMesh,
    //     workFrameSize,
    //     meshGridSize,
    //     4,
    //     mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
    //     -M_PI/6,
    //     true);
    // createLongitudinalTiltWaveSinWarpMesh
    mesh_generator::transverse::createTransverseSinWarpMesh_propX_changeY(
        srcMesh,
        dstMesh,
        workFrameSize,
        meshGridSize, 
        0.5
    );

    // mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveGammaWarpMesh(
        // srcMesh,
        // dstMesh,
        // workFrameSize,
        // meshGridSize,
        // 1.5,
        // 4,
        // mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
        // 0,
        // true);
    // mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveGammaWarpMesh(
    //     dstMesh,    
    //     dstMesh,
    //     workFrameSize,
    //     meshGridSize,
    //     1.5,
    //     1.5,
    //     mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
    //     M_PI/2,
    //     true);
    // cv::Mat srcframe = cv::imread("../data/test_warp_frame_v5.png");
    cv::Mat srcframe = cv::imread("../data/test_warp_frame_v3.png");
    cv::resize(srcframe, srcframe, workFrameSize);
    
    cv::Mat srcMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat dstMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat transformMeshFrame(workFrameSize, CV_8UC3);
    srcMeshFrame = colors::white;
    dstMeshFrame = colors::white;
    transformMeshFrame = colors::white;
    drawMeshTransform(transformMeshFrame, srcMesh, dstMesh, {0,0,0}, {0,0,0}, {0,0,0}, 5, 5);

    drawMesh(transformMeshFrame, srcMesh, colors::blue, true, cv::LineStyles::DASHDOT);
    // drawMesh(transformMeshFrame, dstMesh, colors::red, true, cv::LineStyles::DASHED);

    drawMesh(srcMeshFrame, srcMesh, colors::blue, true, cv::LineStyles::SOLID);
    drawMesh(dstMeshFrame, dstMesh, colors::red, true, cv::LineStyles::SOLID);

    cv::imshow("meshtransform", transformMeshFrame);
    cv::imshow("srcMeshFrame", srcMeshFrame);
    cv::imshow("dstMeshFrame", dstMeshFrame);

    
    cv::imwrite(results_save_dir + "/transformMeshFrame.png", transformMeshFrame);
    cv::imwrite(results_save_dir + "/images_result/srcMeshFrame.png", srcMeshFrame);
    cv::imwrite(results_save_dir + "/images_result/dstMeshFrame.png", dstMeshFrame);

    cv::Mat warpgeomremap, warpperspective, warpaffine;
    MeshWarpApplicator wma(srcMesh, dstMesh);

    wma.apply(srcframe, warpgeomremap);
    imageMeshWarpPerspective(srcframe, warpperspective, dstMesh);
    imageMeshWarpAffine(srcframe, warpaffine, dstMesh);

    cv::imshow("dst_warpgeomremap", warpgeomremap);
    std::cout << "warpgeomremap_size=" << warpgeomremap.size() << std::endl;
    cv::imshow("dst_warpperspective", warpperspective);
    std::cout << "warpperspective_size=" << warpperspective.size() << std::endl;

    cv::imshow("dst_warpaffine", warpaffine);

    cv::imwrite(results_save_dir + "/warpgeomremap.png", warpgeomremap);
    cv::imwrite(results_save_dir + "/warpperspective.png", warpperspective);
    cv::imwrite(results_save_dir + "/warpaffine.png", warpaffine);

    drawMesh(warpgeomremap, dstMesh, colors::red, true, cv::LineStyles::DASHED);
    drawMesh(warpperspective, dstMesh, colors::red, true, cv::LineStyles::DASHED);
    drawMesh(warpaffine, dstMesh, colors::red, true, cv::LineStyles::DASHED);

    cv::imshow("dst_warpgeomremap_mesh", warpgeomremap);
    cv::imshow("dst_warpperspective_mesh", warpperspective);
    cv::imshow("dst_warpaffine_mesh", warpaffine);

    cv::imwrite(results_save_dir + "/warpgeomremap_mesh.png", warpgeomremap);
    cv::imwrite(results_save_dir + "/warpperspective_mesh.png", warpperspective);
    cv::imwrite(results_save_dir + "/warpaffine_mesh.png", warpaffine);

    drawMesh(srcframe, srcMesh, colors::blue, true, cv::LineStyles::DASHED);
    cv::imshow("src", srcframe);
    cv::imwrite(results_save_dir + "/src_mesh.png", srcframe);

    cv::waitKey();
    cv::destroyAllWindows();
}
