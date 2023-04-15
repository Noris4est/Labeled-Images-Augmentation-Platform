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
#include "test_equation_find_pos_map.h"
#include "mesh_grid_nodes_mover_common.hpp"

cv::Scalar red = cv::Scalar(0, 0, 255);
cv::Scalar blue = cv::Scalar(255, 0, 0);
cv::Scalar green = cv::Scalar(0, 255, 0);
cv::Scalar white = cv::Scalar(255, 255, 255);
cv::Scalar black = cv::Scalar(0, 0, 0);
cv::Scalar viol = cv::Scalar(255, 0, 255);
cv::Scalar yang = cv::Scalar(255, 255, 0);
cv::Scalar yell = cv::Scalar(0, 255, 255);
cv::Scalar gray = cv::Scalar(128, 128, 128);

int main(int kargs, char* kwargs[])
{
    cv::Size workFrameSize = {800, 600};
    cv::Size meshGridSize = {32,32};
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
    mesh_generator::longitudinal::createLongitudinalWaveSinFromSourcePointConcentric(
        srcMesh,
        dstMesh,
        workFrameSize,
        meshGridSize,
        cv::Point(300,200), 2); // TODO : разобраться, почему при source_point = {300,300} падает с ошибкой
        // Судя по всему, нужно смещать граничные ноды тоже, т.к. появляется ошибка именно по выпуклости 
        // Возможно снизить требования по выпуклости
    
    MeshWarpApplicator wma(srcMesh, dstMesh);
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

    // cv::imwrite("../images_result/transformMeshFrame.png", transformMeshFrame);
    // cv::imwrite("../images_result/srcMeshFrame.png", srcMeshFrame);
    // cv::imwrite("../images_result/dstMeshFrame.png", dstMeshFrame);

    cv::Mat warpgeomremap, warpperspective, warpaffine;
    
    wma.apply(srcframe, warpgeomremap);
    imageMeshWarpPerspective(srcframe, warpperspective, dstMesh);
    imageMeshWarpAffine(srcframe, warpaffine, dstMesh);

    cv::imshow("dst_warpgeomremap", warpgeomremap);
    std::cout << "warpgeomremap_size=" << warpgeomremap.size() << std::endl;
    cv::imshow("dst_warpperspective", warpperspective);
    std::cout << "warpperspective_size=" << warpperspective.size() << std::endl;

    cv::imshow("dst_warpaffine", warpaffine);

    // cv::imwrite("../images_result/warpgeomremap.png", warpgeomremap);
    // cv::imwrite("../images_result/warpperspective.png", warpperspective);
    // cv::imwrite("../images_result/warpaffine.png", warpaffine);

    drawMesh(warpgeomremap, dstMesh, colors::red, true, cv::LineStyles::DASHED);
    drawMesh(warpperspective, dstMesh, colors::red, true, cv::LineStyles::DASHED);
    drawMesh(warpaffine, dstMesh, colors::red, true, cv::LineStyles::DASHED);

    cv::imshow("dst_warpgeomremap_mesh", warpgeomremap);
    cv::imshow("dst_warpperspective_mesh", warpperspective);
    cv::imshow("dst_warpaffine_mesh", warpaffine);

    // cv::imwrite("../images_result/warpgeomremap_mesh.png", warpgeomremap);
    // cv::imwrite("../images_result/warpperspective_mesh.png", warpperspective);
    // cv::imwrite("../images_result/warpaffine_mesh.png", warpaffine);

    drawMesh(srcframe, srcMesh, colors::blue, true, cv::LineStyles::DASHED);
    cv::imshow("src", srcframe);
    // cv::imwrite("../images_result/src_mesh.png", srcframe);

    cv::waitKey();
    cv::destroyAllWindows();
}
