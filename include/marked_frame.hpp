#ifndef MARKED_FRAME
#define MARKED_FRAME

#include <memory>
#include <opencv2/core.hpp>
#include "annotation.hpp"
#define CV_IMWRITE_JPEG_QUALITY 1

using annotation::Annotation;

class MarkedFrame
{
public:
    MarkedFrame();
    MarkedFrame(cv::Mat frame, Annotation annot);
    void set_frame(const cv::Mat &frame);
    void set_annotation(const Annotation &annot);
    cv::Mat getFrame() const;
    Annotation getAnnot() const;
    MarkedFrame clone() const;
    void resize(cv::Size newsize);
    bool save(const std::string &path2dstDir, 
        const std::string &name,
        cv::Size saveFrameSize = {-1, -1}, 
        const std::string &frameformat = ".jpg");

private:
    cv::Mat frame;
    Annotation annot;
    std::vector<int> params = {CV_IMWRITE_JPEG_QUALITY, 100}; //Параметры imwrite
};

#endif //MARKED_FRAME