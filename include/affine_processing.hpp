#ifndef AFFINE_PROCESSING_HPP
#define AFFINE_PROCESSING_HPP

#include <opencv2/opencv.hpp>

/*
    Применение аффинного преобразования к cv::Point (2i, 2f, 2d)
*/
template <typename P>
P warpAffine2Point(P src, const cv::Mat &warpMat);

/*
    Применение аффинного преобразования к вектору точек cv::Point (2i, 2f, 2d)
*/
template <typename P>
void warpAffine2PointsVec(const std::vector<P> &src, std::vector<P> &dst, const cv::Mat &warpMat);

/*
    Применение аффиного преобразования к cv::Rect
*/
cv::RotatedRect warpAffine2Rect(const cv::Rect2d &src, const cv::Mat &warpMat); // Rect2i REct2d etc
cv::RotatedRect warpAffine2RotatedRect(const cv::RotatedRect &src, const cv::Mat &warpMat);

/*
    Применение аффиного преобразования к вектору cv::Rect
*/
template<typename R>
void warpAffine2RectsVec(const std::vector<R> &src, std::vector<cv::RotatedRect> &dst, const cv::Mat &warpMat);

void ConvertfromAffine2perspectiveMatrix(const cv::Mat &src_affineMat, cv::Mat &dst_perspectiveMat);
#endif //AFFINE_PROCESSING_HPP
