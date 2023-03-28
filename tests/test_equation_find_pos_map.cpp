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
    cv::Mat srcMesh, dstMesh;
    // srcMesh = meshGenerator::creatSqrtWarpMeshFromLeft2Right(dstMesh, cv::Size(600,400), cv::Size(7, 5));
    // srcMesh = meshGenerator::creatSqrtWarpMeshFromTopLeft2BottomRight(dstMesh, workFrameSize, cv::Size(7, 7));
    // srcMesh = meshGenerator::createExpansionWarpMeshFromCenter2BordersSquare(dstMesh, cv::Size(600,400), cv::Size(7, 5));
    // srcMesh = meshGenerator::createExpansionWarpMeshFromCenter2BordersCircle(dstMesh, cv::Size(500,500), cv::Size(5, 3));
//     srcMesh = meshGenerator::createRandomWarpMes(dstMesh, cv::Size(500,500), cv::Size(3, 3));
    srcMesh = meshGenerator::createDistortionWarpMesh(dstMesh, workFrameSize, cv::Size(7, 7));

    MeshWarpApplicator wma(srcMesh, dstMesh);
    cv::Mat srcframe = cv::imread("../data/test_warp_frame_v5.png");
    cv::resize(srcframe, srcframe, workFrameSize);
    
    cv::Mat srcMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat dstMeshFrame(workFrameSize, CV_8UC3);
    cv::Mat transformMeshFrame(workFrameSize, CV_8UC3);
    srcMeshFrame = colors::white;
    dstMeshFrame = colors::white;
    transformMeshFrame = colors::white;
    drawMeshTransform(transformMeshFrame, srcMesh, dstMesh, {0,0,0}, {0,0,0}, {0,0,0}, 5, 5);

    drawMesh(transformMeshFrame, srcMesh, colors::blue, true, cv::LineStyles::DASHDOT);
    drawMesh(transformMeshFrame, dstMesh, colors::red, true, cv::LineStyles::DASHED);

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
    cv::imshow("dst_warpperspective", warpperspective);
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
