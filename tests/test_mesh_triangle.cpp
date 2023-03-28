#include <iostream>
#include "distortion_mesh_generator_zoo.hpp"
#include "warp_mesh_applicator.hpp"
#include "utils.hpp"
#include "colors.hpp"
#include <iomanip>
#include <sstream>

int main(int argc, char* argv[])
{
    std::cout << "hello world" << std::endl;
    cv::Size frameSize = {800, 600};
    cv::Mat primeMesh = meshGenerateTriangle::createPrimeMesh(frameSize, {15, 8});
    cv::Point2i tmp;
    std::ostringstream os;
    for(int i = 0; i< primeMesh.rows; ++i)
    {
        for(int j = 0; j < primeMesh.cols; ++j)
        {
            tmp = primeMesh.at<cv::Point2i>(i, j);
            os << "[" << tmp.x << ", " << tmp.y << "] ";
            std::cout << std::setw(12) << os.str();
            os.str("");
            os.clear();
        }
        std::cout << std::endl;
    }

    cv::Mat meshPicture(frameSize, CV_8UC3);
    drawMesh(meshPicture, primeMesh, colors::red);
    std::vector<cv::Point2i> poly;
    extractPolygon3(primeMesh, poly, {4,2});
    cv::fillConvexPoly(meshPicture, poly, colors::green);

    cv::imshow("prime mesh 1", meshPicture);
    cv::waitKey();
    return 0;
}