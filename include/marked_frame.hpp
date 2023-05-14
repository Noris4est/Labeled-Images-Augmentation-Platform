#ifndef MARKED_FRAME
#define MARKED_FRAME

#include <memory>
#include <opencv2/core.hpp>
#include "annotation.hpp"
#include <opencv2/imgproc.hpp>

#define CV_IMWRITE_JPEG_QUALITY 1

using annotation::Annotation;

class MarkedFrame
{
public:
    MarkedFrame();

    MarkedFrame(cv::Mat frame, Annotation annot);

    MarkedFrame(const std::string &path2frame, const std::string &path2annot);

    cv::Size2i size() const;
    
    void set_frame(const cv::Mat &frame);
    void set_annotation(const Annotation &annot);

    const cv::Mat& getFrameRef() const;
    const Annotation& getAnnotRef() const;

    cv::Mat& getFrameRef();
    Annotation& getAnnotRef();

    cv::Mat getFrame() const;
    Annotation getAnnot() const;

    MarkedFrame clone() const;

    bool isFrameEmpty() const;
    void resize(cv::Size newsize);

    void resize_save_proportions(
        cv::Size dst_size, 
        cv::Scalar free_color = cv::Scalar(0,0,0), 
        cv::InterpolationFlags interpolation = cv::InterpolationFlags::INTER_CUBIC,
        double intersectionAreaThreshold = 0.7);

    void set_frame_and_annot_path(const std::string &path2frame, const std::string &path2annot);

    bool save(const std::string &path2dstDir, 
        const std::string &name,
        cv::Size saveFrameSize = {-1, -1}, 
        const std::string &frameformat = "jpg");

private:
    cv::Mat frame;
    Annotation annot;
    std::vector<int> params = {CV_IMWRITE_JPEG_QUALITY, 100}; //Параметры imwrite
};

#endif //MARKED_FRAME