#include <iostream>
#include "opencv2/opencv.hpp"
#include "cv_resize_tools.hpp"


int main(int argc, char* argv[])
{
    cv::Mat receive_frame = cv::imread("../data/person.jpg");
    cv::Size process_frame_size = {600, 400};
    cv::Size dst_frame_size = {416, 416}; 
    std::cout << "original frames size = " << receive_frame.size() << std::endl;
    cv::Mat process_frame;
    cv::resize(receive_frame, process_frame, process_frame_size);
    cv::Mat resized_save_props_frame;
    cv::resize_save_proportions(process_frame, resized_save_props_frame, dst_frame_size);
    cv::imshow("resized_save_props_win", resized_save_props_frame);
    cv::waitKey(0);
    cv::destroyAllWindows();
return 0;
}
