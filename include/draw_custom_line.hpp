#ifndef DRAW_CUSTOM_LINE
#define DRAW_CUSTOM_LINE

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace cv{
    enum LineStyles
    {
        SOLID,
        DOTTED,
        DASHED,
        DASHDOT
    };
    
    void dotted_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step = 15);
    void dashed_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step = 10);
    void dashdot_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step = 20);
    void dashdot_marker_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step = 20, int markertype = cv::MarkerTypes::MARKER_STAR, int relative_marker_size = 5);

    void line_custom(
        cv::Mat &img, cv::Point p1, cv::Point p2, 
        const cv::Scalar &color, int thickness, int linetype, int linestyle);
    
}
#endif //DRAW_CUSTOM_LINE

