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
#include "marked_frame.hpp"

int main(int kargs, char* kwargs[])
{
    cv::Size workFrameSize = {800, 800};
    cv::Size meshGridSize = {9,9};
    std::string path2srcFrame = "../data/test_labeled_images/000000000139.jpg";
    std::string path2srcAnnot = "../data/test_labeled_images/000000000139.txt";

    cv::Mat srcMesh, dstMesh, proxyMesh;

    srcMesh = mesh_generator::createPrimeMesh(workFrameSize, meshGridSize);

    mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveGammaWarpMesh(
        srcMesh,
        dstMesh,
        workFrameSize,
        meshGridSize,
        1.4,
        2,
        mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX,
        -M_PI/3,
        true);
    
    MarkedFrame process_marked_frame(path2srcFrame, path2srcAnnot);
    process_marked_frame.resize(workFrameSize);
    
    cv::Mat srcMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat dstMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat transformMeshFrame(workFrameSize, CV_8UC3);

    srcMeshFrame = colors::white;
    dstMeshFrame = colors::white;
    transformMeshFrame = colors::white;

    drawMeshTransform(transformMeshFrame, srcMesh, dstMesh, {0,0,0}, {0,0,0}, {0,0,0}, 5, 5);

    drawMesh(transformMeshFrame, srcMesh, colors::blue, true, cv::LineStyles::DASHDOT);
    drawMesh(srcMeshFrame, srcMesh, colors::blue, true, cv::LineStyles::SOLID);
    drawMesh(dstMeshFrame, dstMesh, colors::red, true, cv::LineStyles::SOLID);

    cv::imshow("meshtransform", transformMeshFrame);
    cv::imshow("srcMeshFrame", srcMeshFrame);
    cv::imshow("dstMeshFrame", dstMeshFrame);

    // cv::imwrite("../images_result/transformMeshFrame.png", transformMeshFrame);
    // cv::imwrite("../images_result/srcMeshFrame.png", srcMeshFrame);
    // cv::imwrite("../images_result/dstMeshFrame.png", dstMeshFrame);

    MarkedFrame warpgeomremap, warpperspective, warpaffine;
    MeshWarpApplicator wma(srcMesh, dstMesh);

    wma.apply(process_marked_frame, warpgeomremap);

    // imageMeshWarpPerspective(
    //     process_marked_frame.getFrameRef(), 
    //     warpperspective.getFrameRef(), 
    //     dstMesh);

    // imageMeshWarpAffine(
    //     process_marked_frame.getFrameRef(), 
    //     warpaffine.getFrameRef(), 
    //     dstMesh);

    cv::imshow("dst_warpgeomremap", warpgeomremap.getFrameRef());
    std::cout << "warpgeomremap_size=" << warpgeomremap.size() << std::endl;
    // cv::imshow("dst_warpperspective", warpperspective.getFrameRef());
    // std::cout << "warpperspective_size=" << warpperspective.size() << std::endl;

    // cv::imshow("dst_warpaffine", warpaffine.getFrameRef());

    // cv::imwrite("../images_result/warpgeomremap.png", warpgeomremap);
    // cv::imwrite("../images_result/warpperspective.png", warpperspective);
    // cv::imwrite("../images_result/warpaffine.png", warpaffine);

    drawMesh(warpgeomremap.getFrameRef(), dstMesh, colors::red, true, cv::LineStyles::DASHED);
    drawAnnot(warpgeomremap, colors::blue);
    // drawMesh(warpperspective.getFrameRef(), dstMesh, colors::red, true, cv::LineStyles::DASHED);
    // drawMesh(warpaffine.getFrameRef(), dstMesh, colors::red, true, cv::LineStyles::DASHED);

    cv::imshow("dst_warpgeomremap_mesh", warpgeomremap.getFrameRef());
    // cv::imshow("dst_warpperspective_mesh", warpperspective.getFrameRef());
    // cv::imshow("dst_warpaffine_mesh", warpaffine.getFrameRef());

    // cv::imwrite("../images_result/warpgeomremap_mesh.png", warpgeomremap);
    // cv::imwrite("../images_result/warpperspective_mesh.png", warpperspective);
    // cv::imwrite("../images_result/warpaffine_mesh.png", warpaffine);

    drawMesh(process_marked_frame.getFrameRef(), srcMesh, colors::blue, true, cv::LineStyles::DASHED);
    drawAnnot(process_marked_frame, colors::blue);

    cv::imshow("src", process_marked_frame.getFrameRef());
    // cv::imwrite("../images_result/src_mesh.png", srcframe);

    cv::waitKey();
    cv::destroyAllWindows();
}
