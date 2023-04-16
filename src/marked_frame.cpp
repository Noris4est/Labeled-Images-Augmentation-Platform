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

void MarkedFrame::set_frame(const cv::Mat &frame)
{
    this->frame = frame.clone();
}

void MarkedFrame::set_annotation(const Annotation &annot)
{
    this->annot = annot;
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
    std::string imgDstPath = path2dstDir + std::string("/") + name + std::string(".") + frameformat;

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

void MarkedFrame::resize(cv::Size newsize)
{
    cv::resize(frame, frame, newsize, 0, 0, cv::INTER_CUBIC);
    //меняется размер фрейма, аннотация по умолчанию безразмерна
}