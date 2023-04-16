#include "single_line_annotation.hpp"

namespace annotation
{
    SingleLineAnnotation::SingleLineAnnotation()
    {

    }

    SingleLineAnnotation::SingleLineAnnotation(int class_id, double xc, double yc, double w, double h)
    : class_id(class_id), xc(xc), yc(yc), w(w), h(h)
    {

    }

    SingleLineAnnotation::SingleLineAnnotation(LabledRelBbox lrbbox)
    : class_id(lrbbox.class_id), w(lrbbox.bbox.width), h(lrbbox.bbox.height)
    {
        xc = lrbbox.bbox.x + lrbbox.bbox.width/2;
        yc = lrbbox.bbox.y + lrbbox.bbox.height/2;
    }

    SingleLineAnnotation::SingleLineAnnotation(cv::Rect2d rect, int class_id) : class_id(class_id), h(rect.height), w(rect.width)   
    {
        xc = rect.x + rect.width/2;
        yc = rect.y + rect.height/2;
    }

    LabledRelBbox SingleLineAnnotation::to_lrbbox() const
    {
        LabledRelBbox lrbbox = LabledRelBbox();
        lrbbox.class_id = class_id;
        lrbbox.bbox.width = w;
        lrbbox.bbox.height = h;
        lrbbox.bbox.x = xc - w/2;
        lrbbox.bbox.y = yc - h/2;
        return lrbbox;
    }

    cv::Rect2i SingleLineAnnotation::to_rect_2i(cv::Size frame_size) const
    {
        double rel_x_tl = xc - w / 2;
        double rel_y_tl = yc - h / 2;

        int abs_x_tl = std::round(rel_x_tl * frame_size.width);
        int abs_y_tl = std::round(rel_y_tl * frame_size.height);
        int abs_w = std::round(w * frame_size.width);
        int abs_h = std::round(h * frame_size.height);
        return cv::Rect2i{abs_x_tl, abs_y_tl, abs_w, abs_h};
    }

    std::string SingleLineAnnotation::to_str() const
    {
        std::string line = "";
        std::string space = " ";
        line += std::to_string(class_id) + space;
        line += std::to_string(xc) + space;
        line += std::to_string(yc) + space;
        line += std::to_string(w) + space;
        line += std::to_string(h);
        return line;
    }

    std::tuple<int,double,double,double,double> SingleLineAnnotation::get() const
    {
        return {class_id,xc,yc,w,h};
    }

    cv::Rect2d SingleLineAnnotation::get_rect() const
    {
        return cv::Rect2d(xc - w/2, yc - h/2, w, h);
    }

    void SingleLineAnnotation::set_rect(cv::Rect2d rect)
    {
        xc = rect.x + rect.width/2;
        yc = rect.y + rect.height/2;
        w = rect.width;
        h = rect.height;
    }

    int SingleLineAnnotation::get_class_id()
    {
        return class_id;
    }

    bool SingleLineAnnotation::CheckValidAndAdaptation()
    {
        bool valid = true;
        if (xc + w/2 >= 1)
        {
            valid = false;
            w = adaptationPrecision * 2 * (1 - xc);
        }

        if (yc + h/2 >= 1)
        {
            valid = false;
            h = adaptationPrecision * 2 * (1 - yc);
        }
        if (xc - w/2 <= 0)
        {
            valid = false;
            w = adaptationPrecision * 2 * xc;
        }
        if (yc - h/2 <= 0)
        {
            valid = false;
            h = adaptationPrecision * 2 * yc;
        }
        return valid;
    }

    SingleLineAnnotation& SingleLineAnnotation::operator=(const SingleLineAnnotation& other)
    {
        if(this!= &other)
        {
            class_id = other.class_id;
            xc = other.xc;
            yc = other.yc;
            w = other.w;
            h = other.h;
        }
        return *this;
    }

}