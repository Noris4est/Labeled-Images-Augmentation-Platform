#ifndef CV_RESIZE_TOOLS_HPP
#define CV_RESIZE_TOOLS_HPP

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include "colors.hpp"

namespace cv{
    void resize_save_proportions(
        const cv::Mat &src, 
        cv::Mat &dst, 
        cv::Size dst_size, 
        cv::Scalar free_color = colors::gray, 
        cv::InterpolationFlags interpolate = cv::InterpolationFlags::INTER_CUBIC);
}
#endif // CV_RESIZE_TOOLS_HPP
