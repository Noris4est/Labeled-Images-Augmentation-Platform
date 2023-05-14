#include "marked_frame.hpp"
#ifdef USE_FILESYSTEM
    #include <filesystem>
#endif
#include "path_processing.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#define CV_IMWRITE_JPEG_QUALITY 1

MarkedFrame::MarkedFrame()
{

}

MarkedFrame::MarkedFrame(cv::Mat frame, Annotation annot)
    : frame(frame), annot(annot)
{
    
}

MarkedFrame::MarkedFrame(const std::string &path2frame, const std::string &path2annot)
{
    set_frame_and_annot_path(path2frame, path2annot);
}


cv::Size2i MarkedFrame::size() const
{
    return frame.size();
}

void MarkedFrame::set_frame(const cv::Mat &frame)
{
    this->frame = frame.clone();
}

void MarkedFrame::set_annotation(const Annotation &annot)
{
    this->annot = annot;
}

const cv::Mat& MarkedFrame::getFrameRef() const
{
    return frame;
}

 const Annotation& MarkedFrame::getAnnotRef() const
{
    return annot;
}

cv::Mat &MarkedFrame::getFrameRef()
{
    return frame;
}

Annotation &MarkedFrame::getAnnotRef()
{
    return annot;
}

cv::Mat MarkedFrame::getFrame() const
{
    return frame.clone();
}

Annotation MarkedFrame::getAnnot() const
{
    return annot;
}

MarkedFrame MarkedFrame::clone() const
{
    auto frame = this->getFrame();
    auto annot = this->getAnnot();
    return MarkedFrame(frame, annot);
}

bool MarkedFrame::save(const std::string &path2dstDir, 
    const std::string &name,
    cv::Size saveFrameSize, 
    const std::string &frameformat)
{
    if(!path_processing::isDirExist(path2dstDir))
    {
        return false;
        std::cout << "Warning: directory : " << path2dstDir << "is not exits!" << std::endl;
    }
    annot.saveTxt(path2dstDir, name);
    std::string imgDstPath = path2dstDir + "/" + name + "." + frameformat;

    cv::Mat saveFrame;
    if(saveFrameSize.width >=0 && saveFrameSize.height >=0 && saveFrameSize != frame.size())
    {
        cv::resize(frame, saveFrame, saveFrameSize, 0, 0, cv::INTER_CUBIC);
    }
    else
    {
        saveFrame = frame.clone();
    }

    cv::imwrite(imgDstPath, saveFrame, params);

    return true;
}

bool MarkedFrame::isFrameEmpty() const
{
    return frame.empty();
}

void MarkedFrame::resize(cv::Size newsize)
{
    if(frame.size() != newsize)
    {
        cv::resize(frame, frame, newsize, 0, 0, cv::INTER_CUBIC);
    }
    //меняется размер фрейма, аннотация по умолчанию безразмерна    
}

void MarkedFrame::resize_save_proportions(
    cv::Size dst_size, 
    cv::Scalar free_color, 
    cv::InterpolationFlags interpolation,
    double intersectionAreaThreshold)
{
        assert(!frame.empty());
        assert(dst_size.width >0 && dst_size.height > 0);

        double resize_factor_width = static_cast<double>(dst_size.width) / frame.cols;
        double resize_factor_height = static_cast<double>(dst_size.height) / frame.rows;

        double resize_factor;
        cv::Size dst_size_tmp;
        cv::Mat dst_frame_tmp;
        int x_tl, y_tl;
        if(resize_factor_height < resize_factor_width)
        {
            resize_factor = resize_factor_height;
            dst_size_tmp = {static_cast<int>(resize_factor * frame.cols), dst_size.height};
            x_tl = (dst_size.width - dst_size_tmp.width) / 2;
            y_tl = 0;
        }
        else
        {
            resize_factor = resize_factor_width;
            dst_size_tmp = {dst_size.width, static_cast<int>(resize_factor * frame.rows)};
            x_tl = 0;
            y_tl = (dst_size.height - dst_size_tmp.height) / 2;
        }
        cv::resize(frame, dst_frame_tmp, dst_size_tmp, 0, 0, interpolation);
        annot.putSrcMarkFrame2dstFrame_markRecalculation(dst_size_tmp, dst_size, cv::Point(x_tl, y_tl), intersectionAreaThreshold);
        cv::Mat dst_proxy = cv::Mat(dst_size, frame.type());
        dst_proxy = free_color;
        dst_frame_tmp.copyTo(dst_proxy(cv::Rect(x_tl,y_tl,dst_frame_tmp.cols, dst_frame_tmp.rows)));
        frame = dst_proxy;
}

void MarkedFrame::set_frame_and_annot_path(
    const std::string &path2frame, 
    const std::string &path2annot)
{
    assert(path_processing::isRegularFileExist(path2frame));
    assert(path_processing::isRegularFileExist(path2annot));
    frame = cv::imread(path2frame);
    assert(!frame.empty());
    annot.readTxt(path2annot);
}
