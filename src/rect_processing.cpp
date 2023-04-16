#include "rect_processing.hpp"

#include "affine_processing.hpp"

using namespace rect_processing;
/*
    Расчет площади пересечения largeBbox и smallBbox в долях от площади smallBbox;
*/
double calcIntersectionAreaPart(const cv::Rect2d &largeBbox, const cv::Rect2d &smallBbox)
{
    double y_top = std::max(largeBbox.y, smallBbox.y);
    double y_bottom = std::min(largeBbox.y + largeBbox.height, smallBbox.y + smallBbox.height);
    double x_left = std::max(largeBbox.x, smallBbox.x);
    double x_right = std::min(largeBbox.x + largeBbox.width, smallBbox.x + smallBbox.width);
    if(y_top > y_bottom || x_right < x_left)
        return 0;
    cv::Rect2d intersectionRect = cv::Rect2d(cv::Point2d(x_left, y_top), cv::Point2d(x_right, y_bottom));
    return smallBbox.area() != 0 ? intersectionRect.area() / smallBbox.area() : 0;
} // -- END calcIntersectionAreaPart

IntersectionType checkIntersectionRectInPolygon(std::vector<cv::Point2f> polygon, cv::Rect rect)
{
    return(checkIntersectionRectInPolygon(
        polygon, 
        cv::RotatedRect(
            getRotateRectFromRect(rect)
        )));
} // -- END checkIntersectionRectInPolygon

IntersectionType checkIntersectionRectInPolygon(std::vector<cv::Point2f> polygon, cv::RotatedRect rrect)
{
    std::vector<cv::Point> polygon_2i; //т.к. fillPoly работает только с вектором cv::Point2i
    for(auto p : polygon)
        polygon_2i.push_back(cv::Point(p));

    //предполагаем, что нет отрицательных координат у вершин polygon и rect
    int max_x = polygon[0].x;
    int max_y = polygon[0].y;
    for (auto p : polygon)
    {
        if(p.x > max_x)
            max_x = p.x;
        if(p.y > max_y)
            max_y = p.y;
    }
    cv::Point2f pts[4];
    rrect.points(pts);

    for(int i = 0; i < 4; ++i)
    {
        if(pts[i].x > max_x)
            max_x = pts[i].x;
        if(pts[i].y > max_y)
            max_y = pts[i].y;
    }

    cv::Mat frame1(cv::Size(max_x+1, max_y+1), CV_8UC1);
    frame1 = cv::Scalar();
    cv::Mat frame2 = frame1.clone();
    cv::fillPoly(frame1, polygon_2i, cv::Scalar(255));
    int polygonArea = cv::countNonZero(frame1);
    cv::rectangle(frame2, rrect, cv::Scalar(255), -1);
    int rectArea = cv::countNonZero(frame2);
    cv::Mat frame3;
    cv::bitwise_and(frame1, frame2, frame3);
    int intersectionArea = cv::countNonZero(frame3);
    if(rectArea == intersectionArea || intersectionArea != 0)
        return IntersectionType::FULL_INTERSECTION;
    else if (intersectionArea == 0)
        return IntersectionType::NONE_INTERSECTION;
    else
        return IntersectionType::PARTIAL_INTERSECTION;
} // -- END checkIntersectionRectInPolygon

double calculateIOU(const cv::Rect2i &bbox1, const cv::Rect2i &bbox2)
{
    int y_top = std::max(bbox1.y, bbox2.y);
    int y_bottom = std::min(bbox1.y + bbox1.height, bbox2.y + bbox2.height);
    int x_left = std::max(bbox1.x, bbox2.x);
    int x_right = std::min(bbox1.x + bbox1.width, bbox2.x + bbox2.width);
    if(y_top > y_bottom || x_right < x_left)
        return 0;
    cv::Rect2i intersectionRect = cv::Rect2i(cv::Point2i(x_left, y_top), cv::Point2i(x_right, y_bottom));
    
    int area1 = bbox1.area();
    int area2 = bbox2.area();
    int area_intersect = intersectionRect.area();
    int area_union = area1 + area2 - area_intersect;
    double intersection_over_union = static_cast<double>(area_intersect) / area_union;
    return intersection_over_union;
}

cv::Rect getAbsRectFromRel(const cv::Rect2d &origRect, cv::Size frameSize)
{
    return cv::Rect(
        origRect.x * frameSize.width,
        origRect.y * frameSize.height,
        origRect.width * frameSize.width,
        origRect.height * frameSize.height
    );
} // -- END getAbsRectFromRel

cv::Rect2d getRelRectFromAbs(const cv::Rect &origRect, cv::Size frameSize)
{
    return cv::Rect2d(
        (float)origRect.x / frameSize.width,
        (float)origRect.y / frameSize.height,
        (float)origRect.width / frameSize.width,
        (float)origRect.height / frameSize.height
    );
} // -- END getRelRectFromAbs

/*
Функция пересчитывает абсолютные координаты объекта
исходного фрейма в абсолютные координаты этого же объекта на субфрейме.
Если объект выходит за границы srcROI, то это учитывается.
Адаптация subframeObjAbsRect реализована подрезанием.
*/
cv::Rect catchBboxUsingROI(const cv::Rect &srcROI,const cv::Rect &objAbsRect)
{
    auto subframeObjAbsRect = objAbsRect;
    subframeObjAbsRect.x -= srcROI.x;
    subframeObjAbsRect.y -= srcROI.y;
    if(subframeObjAbsRect.x < 0)
        subframeObjAbsRect.x = 0;
    if(subframeObjAbsRect.y < 0)
        subframeObjAbsRect.y = 0;
    if(subframeObjAbsRect.x + subframeObjAbsRect.width > srcROI.width)
        subframeObjAbsRect.width = srcROI.width - subframeObjAbsRect.x - 1;
    if(subframeObjAbsRect.y + subframeObjAbsRect.height > srcROI.height)
        subframeObjAbsRect.height = srcROI.height - subframeObjAbsRect.y - 1;
    return subframeObjAbsRect;
} // -- END catchBboxUsingROI

cv::Rect2d catchBboxUsingROI(const cv::Rect2d &ROI,const cv::Rect2d &objRelRect)
{
    auto bbox = objRelRect;
    bbox.x -= ROI.x;
    bbox.y -= ROI.y;
    if(bbox.x < 0)
        bbox.x = 0;
    if(bbox.y < 0)
        bbox.y = 0;
    if(bbox.x + bbox.width > ROI.width)
        bbox.width = ROI.width - bbox.x;
    if(bbox.y + bbox.height > ROI.height)
        bbox.height = ROI.height - bbox.y;
    bbox.x /= ROI.width;
    bbox.y /= ROI.height;
    bbox.width /= ROI.width;
    bbox.height /= ROI.height;
    return bbox;
} // -- END catchBboxUsingROI

cv::Point getRectCenter(const cv::Rect &src)
{
    return {src.x + src.width/2, src.y + src.height/2};
} // -- END getRectCenter

cv::Point2d getRectCenter(const cv::Rect2d &src)
{
    return {src.x + src.width/2, src.y + src.height/2};
} // -- END getRectCenter

cv::RotatedRect getRotateRectFromRect(const cv::Rect &src)
{
    return cv::RotatedRect(getRectCenter(src), src.size(), 0);
} // -- END getRotateRectFromRect

cv::RotatedRect getRotateRectFromRect(const cv::Rect2d &src)
{
    return cv::RotatedRect(getRectCenter(src), src.size(), 0);
} // -- END getRotateRectFromRect

void cv::rectangle(cv::Mat &frame, const cv::RotatedRect &rect, cv::Scalar color, int thickness, int lineType, int shift)
{
    cv::Point2f pts2f[4];
    rect.points(pts2f);
    if(thickness >= 0)
    {
        if(thickness == 0)
            thickness = 1;
        for (int i = 0; i < 4; i++)
            cv::line(frame, pts2f[i], pts2f[(i+1)%4], color, thickness, lineType, shift);
    }
    else
    {
        cv::Point *pts = new cv::Point[4];
        for(int i = 0; i < 4; ++i)
        pts[i] = pts2f[i];
        cv::fillConvexPoly(frame, pts, 4, color, lineType, shift);
    }
} // -- END cv::rectangle

void checkAndCorrectRect(cv::Rect &ROI, cv::Size framesize)
{
    if(ROI.x < 0)
        ROI.x = 0;
    if(ROI.y < 0)
        ROI.y = 0;
    if(ROI.x + ROI.width >= framesize.width)
        ROI.x = framesize.width - ROI.width - 1;
    if(ROI.y + ROI.height >= framesize.height)
        ROI.y = framesize.height - ROI.height - 1;
} // -- checkAndCorrectRect

void checkAndCorrectRect(cv::Rect2d &ROI, cv::Size framesize)
{
    if(ROI.x < 0)
        ROI.x = 0;
    if(ROI.y < 0)
        ROI.y = 0;
    if(ROI.x + ROI.width >= framesize.width)
        ROI.x = framesize.width - ROI.width;
    if(ROI.y + ROI.height >= framesize.height)
        ROI.y = framesize.height - ROI.height; 
} // -- checkAndCorrectRect

void getRectPoints(const cv::Rect &src, std::vector<cv::Point> &dst, bool clockwise)
{
    cv::Point tl = src.tl(); //top left
    cv::Point br = src.br(); //bottom right
    cv::Point bl = cv::Point(tl.x, br.y); //bottom left
    cv::Point tr = cv::Point(br.x, tl.y); //top right
    dst.clear();
    dst.reserve(4);
    if(clockwise)
        dst = {bl, tl, tr, br};
    else
        dst = {bl, br, tr, tl};
} // -- getRectPoints

void rect_processing::applyPerspectiveAndGetBounding(cv::Rect src, cv::Rect &dst, cv::Mat warpMat)
{
    std::vector<cv::Point2f> warp_poly;
    rect_processing::applyPerspectiveAndGetPolygon(src, warp_poly, warpMat);
    dst = cv::boundingRect(warp_poly);
} // -- END applyPerspectiveAndGetBounding

void rect_processing::applyPerspectiveAndGetBounding(cv::RotatedRect src, cv::Rect &dst, cv::Mat warpMat)
{
    std::vector<cv::Point2f> warp_poly;
    rect_processing::applyPerspectiveAndGetPolygon(src, warp_poly, warpMat);
    dst = cv::boundingRect(warp_poly);
} // -- END applyPerspectiveAndGetBounding

void rect_processing::applyPerspectiveAndGetMinAreaRot(cv::Rect src, cv::RotatedRect &dst, cv::Mat warpMat)
{
    std::vector<cv::Point2f> warp_poly;
    applyPerspectiveAndGetPolygon(src, warp_poly, warpMat);
    dst = cv::minAreaRect(warp_poly);
} // -- END applyPerspectiveAndGetMinAreaRot

void rect_processing::applyPerspectiveAndGetMinAreaRot(cv::RotatedRect src, cv::RotatedRect &dst, cv::Mat warpMat)
{
    std::vector<cv::Point2f> warp_poly;
    applyPerspectiveAndGetPolygon(src, warp_poly, warpMat);
    dst = cv::minAreaRect(warp_poly);
} // -- END applyPerspectiveAndGetMinAreaRot

void rect_processing::applyPerspectiveAndGetPolygon(cv::Rect src, std::vector<cv::Point2f> &dst, cv::Mat warpMat)
{
    std::vector<cv::Point> src_poly;
    getRectPoints(src, src_poly); //извлекаем рект по точкам начиная с bottom left
    std::vector<cv::Point2f> warp_poly;
    
    //перевод src_poly в src_poly_2f
    std::vector<cv::Point2f> src_poly_2f;
    for(const auto &p : src_poly)
        src_poly_2f.push_back(cv::Point2f(p));
    cv::perspectiveTransform(src_poly_2f, warp_poly, warpMat);
    dst = warp_poly;
} // -- END applyPerspectiveAndGetPolygon

void rect_processing::applyPerspectiveAndGetPolygon(cv::RotatedRect src, std::vector<cv::Point2f> &dst, cv::Mat warpMat)
{
    cv::Point2f pts[4];
    src.points(pts);
    std::vector<cv::Point2f> src_poly = {};
    for(int i = 0; i < 4; ++i)
        src_poly.push_back(cv::Point2f(pts[i]));
    std::vector<cv::Point2f> warp_poly;
    cv::perspectiveTransform(src_poly, warp_poly, warpMat);
    dst = warp_poly;
} // -- END applyPerspectiveAndGetPolygon

template <typename T>
T scaleRect(T rect, float coeff)
{
    T s_rect(rect.x * coeff, rect.y * coeff, rect.width * coeff, rect.height * coeff);
    return s_rect;
}
