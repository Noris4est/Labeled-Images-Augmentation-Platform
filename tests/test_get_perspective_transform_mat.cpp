#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    std::vector<cv::Point2f> src_pts = {
        {368, 0},
        {384, 0},
        {384, 120},
        {368, 120}};
    std::vector<cv::Point2f> dst_pts = {
        {368, 0},
        {384, 0},
        {384, 120},
        {368, 115}};
    cv::Mat invTransform_Mat = cv::getPerspectiveTransform(dst_pts, src_pts, cv::DecompTypes::DECOMP_SVD);
    std::cout << invTransform_Mat << std::endl;
return 0;
}
