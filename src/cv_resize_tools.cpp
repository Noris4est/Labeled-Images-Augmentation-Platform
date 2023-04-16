#include "cv_resize_tools.hpp"
#include <opencv2/opencv.hpp>

namespace cv
{
    void resize_save_proportions(
        const cv::Mat &src, 
        cv::Mat &dst, 
        cv::Size dst_size, 
        cv::Scalar free_color, 
        cv::InterpolationFlags interpolation)
    {
        assert(!src.empty());
        assert(dst_size.width >0 && dst_size.height > 0);

        double resize_factor_width = static_cast<double>(dst_size.width) / src.cols;
        double resize_factor_height = static_cast<double>(dst_size.height) / src.rows;

        double resize_factor;
        cv::Size dst_size_tmp;
        cv::Mat dst_frame_tmp;
        int x_tl, y_tl;
        if(resize_factor_height < resize_factor_width)
        {
            resize_factor = resize_factor_height;
            dst_size_tmp = {static_cast<int>(resize_factor * src.cols), dst_size.height};
            x_tl = (dst_size.width - dst_size_tmp.width) / 2;
            y_tl = 0;
        }
        else
        {
            resize_factor = resize_factor_width;
            dst_size_tmp = {dst_size.width, static_cast<int>(resize_factor * src.rows)};
            x_tl = 0;
            y_tl = (dst_size.height - dst_size_tmp.height) / 2;
        }
        cv::resize(src, dst_frame_tmp, dst_size_tmp, 0, 0, interpolation);
        cv::Mat dst_proxy = cv::Mat(dst_size, src.type());
        dst_proxy = free_color;
        dst_frame_tmp.copyTo(dst_proxy(cv::Rect(x_tl,y_tl,dst_frame_tmp.cols, dst_frame_tmp.rows)));
        dst = dst_proxy;
    }
}
