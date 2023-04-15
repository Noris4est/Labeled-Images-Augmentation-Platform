#include "distortion_mesh_generator_triangle_zoo.hpp"

#include "opencv2/opencv.hpp"
#include "random_processing.hpp"
#include "math_utils_common.hpp"

namespace meshGenerateTriangle
{
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize)
    {
        assert(meshGridSize.width >= 3 && meshGridSize.width % 2 == 1);
        int framew = frameSize.width, frameh = frameSize.height;

        float step_i = (float)frameh / meshGridSize.height;
        float step_j = (float)framew / ((meshGridSize.width - 1) / 2);
        int i_cur, j_cur, i_target, j_target;
        // mesh содержит узлы полигонов, поэтому у него размерность ...
        
        cv::Mat primeMesh(cv::Size((meshGridSize.width - 1) / 2 + 2, meshGridSize.height + 1), CV_32SC2);

        for(int i = 0; i < primeMesh.rows; ++i) //обходим все вершины mesh
        {
            i_cur = i * step_i; // координаты исходной ноды
            if(i_cur == frameh)
                i_cur -= 1;
            if(i % 2 == 0)
            {
                for(int j = 1; j < primeMesh.cols - 1; ++j)
                {
                    j_cur = step_j/2 + (j-1) * step_j;
                    primeMesh.at<cv::Point2i>(i, j) = {j_cur, i_cur};
                }
                primeMesh.at<cv::Point2i>(i, 0) = {0, i_cur};
                primeMesh.at<cv::Point2i>(i, primeMesh.cols - 1) = {framew - 1, i_cur};
            }
            else
            {
                for(int j = 0; j < primeMesh.cols - 2; ++j)
                {
                    j_cur = j * step_j;
                    primeMesh.at<cv::Point2i>(i, j) = {j_cur, i_cur};
                }
                primeMesh.at<cv::Point2i>(i, primeMesh.cols - 2) = {framew - 1, i_cur};
            }
        }
        return primeMesh;
    }
    cv::Mat createRandomWarpMes(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize)
    {
        cv::Mat primeMesh = meshGenerateTriangle::createPrimeMesh(frameSize, meshGridSize);

        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        float scaleFactor = 0.8;
        int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor * frameSize.height / meshGridSize.height / 2;
        int swing;
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1 - i % 2; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = i_cur + rnd(-heightAmplitude, heightAmplitude); // координаты итоговой ноды
                j_target = j_cur + rnd(-widthAmplitude, widthAmplitude);
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize)
    {
        // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
        cv::Mat primeMesh = meshGenerateTriangle::createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        cv::Point p_tmp;
        int i_cur, j_cur, i_target, j_target;
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols -1 - i % 2; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = i_cur; // координаты итоговой ноды
                j_target = round( std::sqrt((frameSize.width) * j_cur) );
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize) // meshGridSize - именно полигоны
    {
        // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
        cv::Mat primeMesh = meshGenerateTriangle::createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        cv::Point p_tmp;
        int i_cur, j_cur, i_target, j_target;
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1 - i % 2; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = round( std::sqrt((frameSize.height) * i_cur) ); // координаты итоговой ноды
                j_target = round( std::sqrt((frameSize.width) * j_cur) );
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }
    cv::Mat createDistortionWarpMesh(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize)
    {
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;

        float k1 = -0.8;
        float k2 = 0;
        float k3 = 0;
        float p1 = 0;
        float p2 = 0;
        
        float x_rel, y_rel, r_rel, xdist_rel, ydist_rel, multiplicator;
        int xdist, ydist;
        int halfwidth = frameSize.width / 2;
        int halfheight = frameSize.height / 2;
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1 - i % 2; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);

                y_rel = (float)(p_tmp.y - halfheight) / frameSize.height; // относительные координаты камеры обскура
                x_rel = (float)(p_tmp.x - halfwidth) / frameSize.width;
                r_rel = std::sqrt(x_rel * x_rel + y_rel * y_rel);

                multiplicator = 1 + k1*ipow(r_rel,2) + k2*ipow(r_rel,4) + k3*ipow(r_rel,6);
                xdist_rel = x_rel * multiplicator;
                ydist_rel = y_rel * multiplicator;
                xdist_rel += 2*p1*x_rel*y_rel + p2*(ipow(r_rel,2) + 2*ipow(x_rel,2));
                ydist_rel += p1*(ipow(r_rel,2) + 2*ipow(y_rel,2)) + 2*p2*x_rel*x_rel*y_rel;
                
                xdist = (frameSize.width * xdist_rel) + halfwidth;
                ydist = (frameSize.height * ydist_rel) + halfheight;

                warpMesh.at<cv::Point2i>(i, j) = {xdist, ydist};
            }
        }
        return primeMesh;
    }

}
