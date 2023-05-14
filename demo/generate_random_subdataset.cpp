#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include "common_types_project.hpp"
#include "darknet_dataset.hpp"
#include "txt_files_processing.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    std::string path2dataset_framesTxtList = "../data/test_darknet_dataset_pipeline/src/dataset3/dataset3.txt11111";
    std::string path2dst_resized_dataset = "../data/test_darknet_dataset_pipeline/dst/dataset3_resize416_save_proportions/1111";
    cv::Size final_size = {416, 416};
    double dataset_scale_factor = 0.2;

    darknet_dataset::Dataset dataset1;

    assert(path_processing::isDirExist(path2dst_resized_dataset));

    if(!dataset1.setPath2txtFileListPath2datasetFrames(path2dataset_framesTxtList))
    {
        std::cout << "Error: failed to set dataset dir!" << std::endl;
        return 1;
    }
    
    std::cout << "BEGIN check valid" << std::endl;
    if(!dataset1.checkValid())
    {
        std::cout << "Error: failed check valid dataset" << std::endl;
        return 2;
    }
    std::cout << "END check valid" << std::endl;

    darknet_dataset::Dataset stride_dataset;
    dataset1.get_random_subdataset(dataset_scale_factor, stride_dataset);

    stride_dataset.save_resized_dataset(path2dst_resized_dataset, final_size, false);
return 0;
}
