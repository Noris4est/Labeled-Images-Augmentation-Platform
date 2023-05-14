#include "distortion_mesh_generator_random_move.hpp"
#include "random_processing.hpp"

#include <opencv2/opencv.hpp>

namespace mesh_generator::random_move
{
    void createRandomWarpMesh(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize, 
        cv::Size meshGridSize,
        bool random_relative_amplitude,
        double rel_amplitude,
        std::pair<float, float> random_amplitude_range)
    {
        cv::Mat warpMesh_proxy = primeMesh_src.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        if(random_relative_amplitude)
        {
            assert(random_amplitude_range.first >=0 && random_amplitude_range.second <=1 && random_amplitude_range.second >= random_amplitude_range.first);
            rel_amplitude = rnd(random_amplitude_range.first, random_amplitude_range.second);
        }
        int widthAmplitude = rel_amplitude * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = rel_amplitude * frameSize.height / meshGridSize.height / 2;
        
        for(int i = 1; i < warpMesh_proxy.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_proxy.cols - 1; ++j)
            {
                p_tmp = primeMesh_src.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = i_cur + rnd(-heightAmplitude, heightAmplitude); // координаты итоговой ноды
                j_target = j_cur + rnd(-widthAmplitude, widthAmplitude);
                warpMesh_proxy.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        warpMesh_dst = warpMesh_proxy;
    }
}
