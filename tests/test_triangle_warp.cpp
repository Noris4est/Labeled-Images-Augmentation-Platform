#include <iostream>
#include "distortion_mesh_generator_zoo.hpp"
#include "warp_mesh_applicator.hpp"
#include "utils.hpp"
#include "colors.hpp"

void printMatOfPoints2i(const cv::Mat &mat)
{
    cv::Point2i tmp;
    std::ostringstream os;
    for(int i = 0; i< mat.rows; ++i)
    {
        for(int j = 0; j < mat.cols; ++j)
        {
            tmp = mat.at<cv::Point2i>(i, j);
            os << "[" << tmp.x << ", " << tmp.y << "] ";
            std::cout << std::setw(12) << os.str();
            os.str("");
            os.clear();
        }
        std::cout << std::endl;
    }
}

int main(int argc, char* argv[])
{
    cv::Size workFrameSize = {800, 600};
    cv::Size meshGridSize = {15, 7};
    cv::Mat srcMesh, warpMesh;
    // srcMesh = meshGenerateTriangle::createRandomWarpMes(warpMesh, workFrameSize, meshGridSize);
    // srcMesh = meshGenerateTriangle::creatSqrtWarpMeshFromLeft2Right(warpMesh, workFrameSize, meshGridSize);
    //srcMesh = meshGenerateTriangle::creatSqrtWarpMeshFromTopLeft2BottomRight(warpMesh, workFrameSize, meshGridSize);
    srcMesh = meshGenerateTriangle::createDistortionWarpMesh(warpMesh, workFrameSize, meshGridSize);
    cv::Mat warpGridFrame(workFrameSize, CV_8UC3);
    warpGridFrame = colors::black;
    drawMesh3nodes(warpGridFrame, warpMesh, colors::green);
    //drawMesh(warpGridFrame, srcMesh, colors::white);
    cv::Mat tmpFrame;
    std::vector<cv::Point> tmpPoly;
    // cv::namedWindow("sliding_triangles");
    // for(int i = 0; i < meshGridSize.height; ++i)
    // {
    //     for(int j = 0; j < meshGridSize.width; ++j)
    //     {
    //         tmpFrame = warpGridFrame.clone();
    //         extractPolygon3(warpMesh, tmpPoly, {j, i});
    //         cv::fillPoly(tmpFrame, tmpPoly, colors::orange);
    //         imshow("sliding_triangles", tmpFrame);
    //         cv::waitKey();
    //     }
    // }

    // printMatOfPoints2i(warpMesh);
    // printMatOfPoints2i(srcMesh);

    // cv::imshow("warp grid", warpGridFrame);
    
    MeshWarpApplicator wma(srcMesh, warpMesh, workFrameSize, meshGridSize);
    cv::Mat srcFrame = cv::imread("../data/test_warp_frame_v5.png");
    cv::resize(srcFrame, srcFrame, workFrameSize);
    cv::Mat dstFrame;
    wma.apply(srcFrame, dstFrame);
    drawMesh3nodes(dstFrame, warpMesh, colors::red, true);
    cv::imshow("warp frame", dstFrame);
    cv::waitKey();


}