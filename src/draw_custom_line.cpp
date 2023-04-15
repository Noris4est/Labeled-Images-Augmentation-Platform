
#include <opencv2/imgproc.hpp>
#include "vector_creation_routines.hpp"
#include "draw_custom_line.hpp"


void cv::line_custom(
        cv::Mat &img, cv::Point p1, cv::Point p2, 
        const cv::Scalar &color, int thickness, int linetype, int linestyle)
{
    switch (linestyle)
    {
    case cv::LineStyles::SOLID:
    {
        line(img, p1, p2, color, thickness, linetype);
        break;
    }
    case cv::LineStyles::DOTTED:
    {
        dotted_line(img, p1, p2, color, thickness, linetype);
        break;
    }
    case cv::LineStyles::DASHED:
    {
        dashed_line(img, p1, p2, color, thickness, linetype);
        break;
    }
    case cv::LineStyles::DASHDOT:
    {
        dashdot_line(img, p1, p2, color, thickness, linetype);
        break;
    }
    default:
        throw std::runtime_error("Not supported linestyle");
        break;
    }
}

void cv::dotted_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step)
{
    int dotstep = relative_step * thickness;
    auto pts = cplt::arange(p1, p2, dotstep);
    for(const auto &p : pts)
    {
        cv::circle(img, p, thickness, color, -1, linetype);
    }
}   

void cv::dashed_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step)
{
    int dashstep = relative_step * thickness;
    auto pts = cplt::arange(p1, p2, dashstep);
    for(int i = 0; i < pts.size() - 1; ++i)
    {
        if(i % 2 == 0)
        {
            cv::line(img, pts[i], pts[i+1], color, thickness, linetype);
        }
    }
}

void cv::dashdot_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step)
{
    int dashstep = relative_step * thickness;
    auto pts = cplt::arange(p1, p2, dashstep);
    cv::Point internalPoint;
    int x_inter, y_inter;
    for(int i = 0; i < pts.size() - 1; ++i)
    {
        if(i % 2 == 0)
        {

            cv::line(img, pts[i], pts[i+1], color, thickness, linetype);
        }
        else
        {
            x_inter = (pts[i].x + pts[i+1].x) / 2;
            y_inter = (pts[i].y + pts[i+1].y) / 2;
            internalPoint = {x_inter, y_inter};
            cv::circle(img, internalPoint, thickness, color, -1, linetype);
        }
    }
}

void cv::dashdot_marker_line(cv::Mat &img, cv::Point p1, cv::Point p2, const cv::Scalar &color, int thickness, int linetype, int relative_step, int markertype, int relative_marker_size)
{
    int dashstep = relative_step * thickness;
    int markersize = relative_marker_size * thickness;
    auto pts = cplt::arange(p1, p2, dashstep);
    cv::Point internalPoint;
    int x_inter, y_inter;
    for(int i = 0; i < pts.size() - 1; ++i)
    {
        if(i % 2 == 0)
        {

            cv::line(img, pts[i], pts[i+1], color, thickness, linetype);
        }
        else
        {
            x_inter = (pts[i].x + pts[i+1].x) / 2;
            y_inter = (pts[i].y + pts[i+1].y) / 2;
            internalPoint = {x_inter, y_inter};
            // cv::circle(img, internalPoint, thickness, color, -1, linetype);
            cv::drawMarker(img, internalPoint, color, markertype, markersize, thickness, linetype);
        }
    }
}
