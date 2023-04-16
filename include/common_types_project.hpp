#ifndef COMMONTYPESPROJECT_HPP
#define COMMONTYPESPROJECT_HPP

#include <string>
#include <vector>
#include <opencv2/core/types.hpp>
#include "path_processing.hpp"

struct QualityScores
{
    double tp, fp, fn; // for multiplication with float/double weights (primary integers)
    double iou; // intersection over union
    QualityScores()
    {
        tp = 0;
        fp = 0;
        fn = 0;
        iou = 0;
    }
    QualityScores(double tp, double fp, double fn, double iou) : tp(tp), fp(fp), fn(fn), iou(iou)
    {

    }
    QualityScores operator* (double value);
};

QualityScores operator+ (const QualityScores &lhs, const QualityScores &rhs);


struct SingleObjectDetectionResult
{
    int class_id;
    double rel_xc, rel_yc, rel_w, rel_h; // relative values [0..1]
    double confidence;

    cv::Rect2i to_rect_2i(cv::Size framesize) const
    {
        double rel_x_tl = rel_xc - rel_w / 2;
        double rel_y_tl = rel_yc - rel_h / 2;

        int abs_x_tl = std::round(rel_x_tl * framesize.width);
        int abs_y_tl = std::round(rel_y_tl * framesize.height);
        int abs_w = std::round(rel_w * framesize.width);
        int abs_h = std::round(rel_h * framesize.height);
        return cv::Rect2i{abs_x_tl, abs_y_tl, abs_w, abs_h};
    }
};

struct SingleFrameDetectionResult
{
    double net_threshold;
    cv::Size2i frame_size;
    std::vector<SingleObjectDetectionResult> detections;
};


struct LabledRelBbox
{
    int class_id;
    cv::Rect2d bbox;
    LabledRelBbox()
    {
        class_id = 0;
        bbox.x = 0;
        bbox.y = 0;
        bbox.width = 0;
        bbox.height = 0;
    }
    LabledRelBbox(int class_id, cv::Rect2d bbox)
    : class_id(class_id), bbox(bbox)
    {

    }
    bool checkValid()
    {
        if(
            bbox.x < 0 || bbox.x > 1
            || bbox.y < 0 || bbox.y > 1
            || bbox.width < 0 || bbox.width > 1
            || bbox.height < 0 || bbox.height > 1
        )
            return false;
        return true;
    }
}; // rel - relative (rect2d)

#endif //COMMONTYPES_HPP
