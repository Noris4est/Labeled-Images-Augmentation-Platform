#include "vector_creation_routines.hpp"

std::vector<double> cplt::linspace(double start, double end, int num)
{
    if(num <= 0)
        throw std::runtime_error("Error: argument \"num\" is not correct");
    std::vector<double> result;
    result.reserve(num);
    if(num == 1)
    {
        result.push_back(start);
        return result;
    }
    double step = (end - start) / (num - 1);
    for(int i = 0; i < num; ++i)
    {
        result.push_back(start + i*step);
    }
    return result;
}

std::vector<double> cplt::arange(double start, double end, double step)
{
    assert(step !=0 );
    assert(step < 0 && start > end || step > 0 && start < end);

    int size = std::ceil(std::abs((end - start) / step));
    std::vector<double> result;
    if(size  == 1)
    {
        result.push_back(start);
        return result;
    }
    result.reserve(size);
    for(int i = 0; i < size; ++i)
    {
        result.push_back(start + i * step);
    }
    return result; // some action for test "initial commit" 
}

std::vector<cv::Point> cplt::arange(cv::Point start, cv::Point end, double step)
{
    int x_start = start.x;
    int y_start = start.y;
    int x_end = end.x;
    int y_end = end.y;
    int dx = x_end - x_start;
    int dy = y_end - y_start;
    double len = std::sqrt(dx*dx + dy*dy);
    int num = len / step + 1;
    if(num == 1)
    {
        num = 2;
    }
    std::vector<double> x_vec = cplt::linspace(x_start, x_end, num);
    std::vector<double> y_vec = cplt::linspace(y_start, y_end, num);
    std::vector<cv::Point> result;
    result.reserve(num);
    for(int i = 0; i < num; ++i)
    {
        result.push_back(cv::Point(x_vec[i], y_vec[i]));
    }
    return result;
}