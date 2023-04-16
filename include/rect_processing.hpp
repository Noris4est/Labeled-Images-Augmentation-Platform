#ifndef RECT_PROCESSING_HPP
#define RECT_PROCESSING_HPP

#include <opencv2/opencv.hpp>

//Расчет доли пересечения largeBbox и smallBbox в долях smallBbox
double calcIntersectionAreaPart(const cv::Rect2d &largeBbox, const cv::Rect2d &smallBbox);

//Перевод bbox'а из относительных в абсолютные координаты и наоборот
cv::Rect getAbsRectFromRel(const cv::Rect2d &origRect, cv::Size frameSize);
cv::Rect2d getRelRectFromAbs(const cv::Rect &origRect, cv::Size frameSize);

//Расчет координат bbox'а относительно srcROI
cv::Rect catchBboxUsingROI(const cv::Rect &srcROI, const cv::Rect &objAbsRect);
cv::Rect2d catchBboxUsingROI(const cv::Rect2d &ROI, const cv::Rect2d &objRelRect);

cv::Point getRectCenter(const cv::Rect &src);
cv::Point2d getRectCenter(const cv::Rect2d &src);

cv::RotatedRect getRotateRectFromRect(const cv::Rect &src);
cv::RotatedRect getRotateRectFromRect(const cv::Rect2d &src);

//получение точек rect'a начиная с bottom_left с учетом clockwise (по часовой / против часовой)
void getRectPoints(const cv::Rect &src, std::vector<cv::Point> &dst, bool clockwise = true);

//Функция проверяет ROI на корректность координат (чтобы не выходило за границы framesize) и корректирует
void checkAndCorrectRect(cv::Rect &ROI, cv::Size framesize);
void checkAndCorrectRect(cv::Rect2d &ROI, cv::Size framesize);

namespace rect_processing
{
    /*
        Функция применяет перспективное преобразование к src ректу, 
        Преобразование задается матрицей warpMat.
        Семейство функций applyPerspective... вынесено в namespace
        для избежания конфликта имен.
    */
    //Возвращает ближайший rect
    void applyPerspectiveAndGetBounding(cv::Rect src, cv::Rect &dst, cv::Mat warpMat);
    void applyPerspectiveAndGetBounding(cv::RotatedRect src, cv::Rect &dst, cv::Mat warpMat);

    //Возвращает ближайший RotatedRect
    void applyPerspectiveAndGetMinAreaRot(cv::Rect src, cv::RotatedRect &dst, cv::Mat warpMat);
    void applyPerspectiveAndGetMinAreaRot(cv::RotatedRect src, cv::RotatedRect &dst, cv::Mat warpMat);

    //Возвращает полученный полигон 
    void applyPerspectiveAndGetPolygon(cv::Rect src, std::vector<cv::Point2f> &dst, cv::Mat warpMat);
    void applyPerspectiveAndGetPolygon(cv::RotatedRect src, std::vector<cv::Point2f> &dst, cv::Mat warpMat);
    
} // -- END namespace rect_processing

enum class IntersectionType
{
    NONE_INTERSECTION = 0,
    PARTIAL_INTERSECTION = 1,
    FULL_INTERSECTION = 2
};

/*
    Функция расчета пересечения произвольного полинома и rect'a. 
    Возвращает один из трех возможных типов пересечения.
*/
IntersectionType checkIntersectionRectInPolygon(std::vector<cv::Point2f> polygon, cv::Rect rect);
IntersectionType checkIntersectionRectInPolygon(std::vector<cv::Point2f> polygon, cv::RotatedRect rrect);

//Кастомная функция отображения RotatedRect (В стандартном cv отсутствует)
namespace cv
{
    void rectangle(cv::Mat &frame, const cv::RotatedRect &rect, cv::Scalar color, int thickness = 1, int lineType = 8, int shift = 0);
}

// Масштабирование ректа
template <typename T>
T scaleRect(T rect, float coeff);

double calculateIOU(const cv::Rect2i &bbox1, const cv::Rect2i &bbox2);

#endif //RECT_PROCESSING_HPP
