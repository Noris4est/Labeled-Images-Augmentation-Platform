#ifndef VECTOR_CREATION_ROUTINES_HPP
#define VECTOR_CREATION_ROUTINES_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <opencv2/core/types.hpp>

namespace cplt
{
    std::vector<double> linspace(double start, double end, int num);
    std::vector<double> arange(double start, double end, double step);
    std::vector<cv::Point> arange(cv::Point start, cv::Point end, double step);
}

#endif // VECTOR_CREATION_ROUTINES_HPP
