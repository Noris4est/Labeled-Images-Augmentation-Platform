#include "distortion_mesh_generator_transverse_waves.hpp"
#include <opencv2/opencv.hpp>

namespace mesh_generator::transverse
{
    // генерация искаженной сетки, с характерным узором синуса, распространяющегося вдоль x, с изменением координат y сетки
    void createTransverseSinWarpMesh_propX_changeY(
        const cv::Mat primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor)
    {
        cv::Mat warpMesh_proxy = primeMesh_src.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        // int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor * frameSize_src.height / meshGridSize_src.height / 2;
        int meshWidthPeriod = frameSize_src.width / meshGridSize_src.width;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period = 8 ; // 2 - min по теореме Котельникова
        int sin_period = sin_period_divided_grid_period * meshWidthPeriod;
        double sin_arg = 2 * M_PI / sin_period; 
        for(int i = 1; i < warpMesh_proxy.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_proxy.cols - 1; ++j)
            {
                p_tmp = primeMesh_src.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = i_cur + heightAmplitude * std::sin(sin_arg * j_cur); // координаты итоговой ноды
                j_target = j_cur;
                warpMesh_dst.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        warpMesh_dst = warpMesh_proxy;
    }

    void createTransverseSinWarpMesh_propY_changeX(
        const cv::Mat primeMesh_src,
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor)
    {
        cv::Mat warpMesh_proxy = primeMesh_src.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        int widthAmplitude = scaleFactor * frameSize_src.width / meshGridSize_src.width / 2;
        int meshHeightPeriod = frameSize_src.height / meshGridSize_src.height;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period = 8 ; // 2 - min по теореме Котельникова
        int sin_period = sin_period_divided_grid_period * meshHeightPeriod;
        double sin_arg = 2 * M_PI / sin_period; 
        for(int i = 1; i < warpMesh_proxy.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_proxy.cols - 1; ++j)
            {
                p_tmp = primeMesh_src.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                j_target = j_cur + widthAmplitude * std::sin(sin_arg * i_cur); // координаты итоговой ноды
                i_target = i_cur;
                warpMesh_proxy.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        warpMesh_dst = warpMesh_proxy;
    }

    void createTransverseSinWarpMesh_propXY_changeXY(
        const cv::Mat primeMesh_src,
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        float scaleFactor_X,
        float scaleFactor_Y)
    {
        cv::Mat warpMesh_proxy = primeMesh_src.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        int widthAmplitude = scaleFactor_X * frameSize_src.width / meshGridSize_src.width / 2;
        int heightAmplitude = scaleFactor_Y * frameSize_src.height / meshGridSize_src.height / 2;
        int meshWidthPeriod = frameSize_src.width / meshGridSize_src.width;
        int meshHeightPeriod = frameSize_src.height / meshGridSize_src.height;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period_x = 8 ; // 2 - min по теореме Котельникова
        int sin_period_divided_grid_period_y = 8 ; // 2 - min по теореме Котельникова
        int sin_period_y = sin_period_divided_grid_period_y * meshHeightPeriod;
        int sin_period_x = sin_period_divided_grid_period_x * meshWidthPeriod;
        double sin_arg_prop_x = 2 * M_PI / sin_period_x; 
        double sin_arg_prop_y = 2 * M_PI / sin_period_y; 
        for(int i = 1; i < warpMesh_proxy.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_proxy.cols - 1; ++j)
            {
                p_tmp = primeMesh_src.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                j_target = j_cur + widthAmplitude * std::sin(sin_arg_prop_y * i_cur); // координаты итоговой ноды
                i_target = i_cur + heightAmplitude * std::sin(sin_arg_prop_x * j_cur);
                warpMesh_proxy.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        warpMesh_dst = warpMesh_proxy;
    }
}