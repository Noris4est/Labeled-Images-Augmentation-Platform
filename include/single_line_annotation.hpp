#ifndef SINGLE_LINE_ANNOTATION_HPP
#define SINGLE_LINE_ANNOTATION_HPP

#include "common_types_project.hpp"

namespace annotation
{
    class SingleLineAnnotation
    {
    public:
        SingleLineAnnotation();
        SingleLineAnnotation(int class_id, double xc, double yc, double w, double h);
        SingleLineAnnotation(LabledRelBbox lrbbox);
        SingleLineAnnotation(cv::Rect2d, int class_id = 0);
        LabledRelBbox to_lrbbox() const;
        cv::Rect2i to_rect_2i(cv::Size frame_size) const;
        std::string to_str() const;
        std::tuple<int,double,double,double,double> get() const;
        cv::Rect2d get_rect() const ;
        void set_rect(cv::Rect2d rect);
        int get_class_id();
        bool CheckValidAndAdaptation();  // Проверка валидности строки аннотации на предмет небольших искажений и адаптация
        SingleLineAnnotation& operator=(const SingleLineAnnotation& other);
    private:
        int class_id = 0;
        double xc = 0, yc = 0, w = 0, h = 0; //[0...1]
        const double adaptationPrecision = 0.9999;
    };

}

#endif //SINGLE_LINE_ANNOTATION_HPP