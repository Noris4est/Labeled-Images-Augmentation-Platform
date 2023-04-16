#include "affine_processing.hpp"

template <typename P>
P warpAffine2Point(P src, const cv::Mat &warpMat)
{
    auto it = warpMat.begin<double>();
    auto m00 = *it; it++;
    auto m01 = *it; it++;
    auto m02 = *it; it++;
    auto m10 = *it; it++;
    auto m11 = *it; it++;
    auto m12 = *it;
    double x_new = m00*src.x + m01*src.y + m02;
    double y_new = m10*src.x + m11*src.y + m12;
    return P(x_new, y_new);
}
// Here is the explicit instanciation
template cv::Point warpAffine2Point<cv::Point>(cv::Point src,const cv::Mat &warpMat);


template <typename P>
void warpAffine2PointsVec(const std::vector<P> &src, std::vector<P> &dst, const cv::Mat &warpMat)
{
    dst.clear();
    dst.reserve(src.size());
    for(int i = 0; i < src.size(); i++)
        dst.push_back(warpAffine2Point<P>(src[i], warpMat));
}

cv::RotatedRect warpAffine2Rect(const cv::Rect2d &src, const cv::Mat &warpMat)
{
    cv::Point2d center(src.x + src.width/2, src.y + src.height/2);
    cv::RotatedRect r_rect(center, src.size(), 0);
    return warpAffine2RotatedRect(r_rect, warpMat);
}


cv::RotatedRect warpAffine2RotatedRect(const cv::RotatedRect &src, const cv::Mat &warpMat)
{
    cv::Point2f pts[4];
    src.points(pts);
    std::vector<cv::Point2f> ptsOrgVec;
    ptsOrgVec.reserve(4);
    for(int i = 0; i < 4; ++i)
        ptsOrgVec.push_back(pts[i]);
    std::vector<cv::Point2f> ptsWarp;
    ptsWarp.reserve(4);
    warpAffine2PointsVec<cv::Point2f>(ptsOrgVec, ptsWarp, warpMat);
    auto resultRect = cv::minAreaRect(ptsWarp);
    return resultRect;
}

template<typename R>
void warpAffine2RectsVec(const std::vector<R> &src, std::vector<cv::RotatedRect> &dst, const cv::Mat &warpMat)
{
    dst.clear();
    dst.reserve(src.size());
    for(int i = 0; i < src.size(); i++)
        dst.push_back(warpAffine2Rect(src[i], warpMat));
}

void ConvertfromAffine2perspectiveMatrix(const cv::Mat &src_affineMat, cv::Mat &dst_perspectiveMat)
{
    dst_perspectiveMat = cv::Mat::eye(cv::Size(3,3), src_affineMat.type());
    src_affineMat.copyTo(dst_perspectiveMat(cv::Rect(0,0,3,2)));
}



