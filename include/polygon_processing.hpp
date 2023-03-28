#ifndef POLYGON_PROCESSING_HPP
#define POLYGON_PROCESSING_HPP

#include "opencv2/core/types.hpp"
#include <vector>
#include "colors.hpp"

bool isConvexPolygon(
    const std::vector<cv::Point> &contour,
    bool checkSelfIntersection = true, 
    bool validZeroVectorsProduct = false);
    
std::vector<cv::Point> createPentagramPoly(
    cv::Point center, 
    int externalRadius,
    float phase); // phase [rad]
    
void customDrawPoly(
    cv::Mat &frame,
    const std::vector<cv::Point> &poly, 
    cv::Scalar polyColor, 
    int polyThickness = 1, 
    int textOffset = 20,
    float fontScale = 0.7,
    cv::Scalar textColor = colors::blue,
    int shift = 0);
#endif //POLYGON_PROCESSING_HPP