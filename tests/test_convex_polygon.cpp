#include <iostream>
#include <opencv2/opencv.hpp>
#include <polygon_processing.hpp>
#include <colors.hpp>
#include <frametext.hpp>


int main(int kargs, char *kwargs[])
{
    auto pent = createPentagramPoly({300,300},100,0);

    std::vector<cv::Point> poly1{{100, 100}, {150, 100}, {200,100}, {200, 200}, {100,200}};
    std::vector<cv::Point> poly2{{100, 100}, {300,100}, {250, 200}, {30, 240}};
    auto poly = poly1;
    for(auto p : poly)
        std::cout << p << ", ";
    std::cout << std::endl;
    // std::reverse(poly.begin(), poly.end());
    std::rotate(poly.begin(), poly.begin() + 2, poly.end());
    std::cout << "after preprocess poly" << std::endl;
    for(auto p : poly)
        std::cout << p << ", ";
    std::cout << std::endl;
    bool isConvex = isConvexPolygon(poly, true, false);
    std::cout << "isConvex = " << isConvex << std::endl;
    
    cv::Mat frame(cv::Size(1000,800), CV_8UC3);
    frame = colors::black;
    customDrawPoly(frame, poly, colors::red);
    
    cv::namedWindow("win1");
    cv::imshow("win1", frame);
    cv::waitKey();
    cv::destroyAllWindows();
    
}
