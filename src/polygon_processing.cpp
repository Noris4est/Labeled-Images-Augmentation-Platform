#include <polygon_processing.hpp>

#include <vector>
#include <opencv2/opencv.hpp>
#include <frametext.hpp>

/*
    Проверяется полигон на выпуклость.
    1. Проверяется постоянство знака векторного произведения 
        последовательно рассматриваемых ребер контура.
    2. Нулевое векторное произведение обрабатывается в соответствии 
        с передаваемой настройкой validZeroVectorsProduct.
        При рассмотрении полигона в форме прямоугольника
        обеспечивается постоянство знака векторного произведения,
        однако, если в любое ребро осуществить вставку промежуточного
        узла, то для такого полигона настройка определяет, 
        будет ли он считаться выпукым. Поскольку на ребре
        с паразитным узлом будет нулевое векторное произведение (sin(0)=0).
    3. При установке флага checkSelfIntersection = True функция проверяет 
        полигон на предмет самопересечений. При активации настройки усложняется
        вычислительная сложность алгоритма. Настоящий флаг позволяет
        детектировать пентограммо-подрные полигоны. В таких полигонах
        обеспечивается постоянство знака векторного произведения,
        но по существу они не являются выпуклыми. В библиотечной функции
        cv::isContourConvex не осуществляется проверка пентограммо-подобность.
        В официальном описании к функции " The contour must be simple, 
        that is, without self-intersections. 
        Otherwise, the function output is undefined." Проверка на самопересечение
        осуществляется сверкой суммарных углов, 
        вычисленными экспериментально и аналитически.
*/
bool isConvexPolygon(
    const std::vector<cv::Point> &contour,
    bool checkSelfIntersection, 
    bool validZeroVectorsProduct)
{
    std::function<double(cv::Point, cv::Point, cv::Point, cv::Point)> crossProduct = 
        [](cv::Point vec1begin, cv::Point vec1end, cv::Point vec2begin, cv::Point vec2end){
            int dx1 = vec1end.x - vec1begin.x;
            int dy1 = vec1end.y - vec1begin.y;
            int dx2 = vec2end.x - vec2begin.x;
            int dy2 = vec2end.y - vec2begin.y;
            return dx1 * dy2 - dy1 * dx2;
        };
    std::function<double(cv::Point, cv::Point, cv::Point, cv::Point)> scalarProduct = 
        [](cv::Point vec1begin, cv::Point vec1end, cv::Point vec2begin, cv::Point vec2end){
            int dx1 = vec1end.x - vec1begin.x;
            int dy1 = vec1end.y - vec1begin.y;
            int dx2 = vec2end.x - vec2begin.x;
            int dy2 = vec2end.y - vec2begin.y;
            return dx1 * dx2 + dy1 * dy2;
        };

    std::function<float(cv::Point, cv::Point)> vectorLength = 
        [](cv::Point p1, cv::Point p2){
            int dx = p2.x - p1.x;
            int dy = p2.y - p1.y;
            return std::sqrt(dx * dx + dy * dy);
        };
    int csize = contour.size();
    assert(csize >= 3);
    cv::Point p_prev2 = contour[csize-2];
    cv::Point p_prev1 = contour[csize-1];
    cv::Point p_cur;
    int positive_cr_pr_count = 0;
    int negative_cr_pr_count = 0;
    int cr_pr, sc_pr;
    float angleCurrent; 
    float angleTotal = 0;
    float len1, len2;
    for(int i = 0; i < csize; ++i)
    {   
        p_cur = contour[i];
        cr_pr = crossProduct(p_prev2, p_prev1, p_prev1, p_cur);
        if(cr_pr > 0)
        {
            ++positive_cr_pr_count;
        }
        else if(cr_pr < 0)
        {
            ++negative_cr_pr_count;
        }
        else
        {
            if(!validZeroVectorsProduct)
            {
                return false;  // не выпуклый
            }
        }
        if(checkSelfIntersection)
        {
            sc_pr = scalarProduct(p_prev2, p_prev1, p_prev1, p_cur);
            len1 = vectorLength(p_prev2, p_prev1);
            len2 = vectorLength(p_prev1, p_cur);
            angleCurrent = std::acos(sc_pr / (len1 * len2));
            if(cr_pr == 0)
            {
                angleTotal += M_PI;
            }
            else
            {
                angleTotal += M_PI - angleCurrent;
            }
        }
        p_prev2 = p_prev1;
        p_prev1 = p_cur;
    }

    if(positive_cr_pr_count != 0 && negative_cr_pr_count != 0)
    {
        return false;
    }

    //проверка на сумму positive_cr_pr_count + positive_cr_pr_count, тк для валидности необходимо, чтобы неколлиниарных было хотя бы 3

    int non180_angles_count = positive_cr_pr_count != 0 ? positive_cr_pr_count : negative_cr_pr_count;
    if(non180_angles_count < 3)
    {
        return false;
    }

    //если включена проверка на самопересечения
    if(checkSelfIntersection)
    {
        float necessaryAngleTotal = (csize - 2) * M_PI; //аналитическая формула из классической геометрии
        float tolerance = 0.05;
        if( (necessaryAngleTotal - angleTotal) / necessaryAngleTotal > tolerance)
        {
            return false;
        }
    }
    
    return true;
}


std::vector<cv::Point> createPentagramPoly(
    cv::Point center, 
    int externalRadius,
    float phase) // phase [rad]
{
    std::vector<cv::Point> poly;
    poly.reserve(5); // number of nodes
    float angleStep = 2 * M_PI / 5;
    float phaseShift = phase - M_PI_2;
    float currentAngle = phaseShift;
    int xCur, yCur;
    for(int i = 0; i < 5; ++i)
    {
        xCur = center.x + externalRadius * std::cos(currentAngle);
        yCur = center.y + externalRadius * std::sin(currentAngle);
        poly.push_back(cv::Point(xCur, yCur));
        currentAngle += angleStep;
    }
    std::vector<uint8_t> id_sequence = {0, 3, 1, 4, 2}; //0, 3, 1, 4, 2}; {0, 2, 4, 1, 3}
    std::vector<cv::Point> pentagram;
    pentagram.reserve(5);
    for(int i = 0; i < 5; ++i)
    {
        pentagram.push_back(poly[id_sequence[i]]);
    }
    return pentagram;
}

void customDrawPoly(
    cv::Mat &frame,
    const std::vector<cv::Point> &poly, 
    cv::Scalar polyColor, 
    int polyThickness, 
    int textOffset,
    float fontScale ,
    cv::Scalar textColor,
    int shift)
{
    int linetype = cv::LineTypes::LINE_AA;
    int fontFace = cv::FONT_HERSHEY_COMPLEX;
    cv::polylines(frame, poly, true, polyColor, polyThickness, linetype);
    float xSum = 0, ySum = 0;
    for(auto const p : poly)
    {
        xSum += p.x;
        ySum += p.y;
    }
    cv::Point2f centerMass = {xSum / poly.size(), ySum / poly.size()};
    cv::Point pCur;
    cv::Point textPcur;
    float dxCenter2cur, dyCenter2cur, lenCenter2cur;
    float dxOffset, dyOffset;
    for(int i = 0; i < poly.size(); ++i)
    {
        pCur = poly[i];
        dxCenter2cur = pCur.x - centerMass.x;
        dyCenter2cur = pCur.y - centerMass.y;
        lenCenter2cur = std::sqrt(dxCenter2cur * dxCenter2cur + dyCenter2cur * dyCenter2cur);
        dxOffset = dxCenter2cur / lenCenter2cur * textOffset;
        dyOffset = dyCenter2cur / lenCenter2cur * textOffset;
        textPcur = {pCur.x + (int)dxOffset, pCur.y + (int)dyOffset};
        cv::circle(frame, pCur, 3, textColor, 1);
        frameText::putText(frame, std::to_string(i), frameText::corner::center, textPcur, 0, fontFace, fontScale, textColor, 1, linetype);
    }
}
