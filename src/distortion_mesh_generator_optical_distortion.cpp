#include "distortion_mesh_generator_optical_distortion.hpp"

#include <opencv2/opencv.hpp>
#include "math_utils_common.hpp"

namespace mesh_generator::optical_distortion
{

    void createDistortionWarpMesh(
        const cv::Mat primeMesh_src, 
        cv::Mat &warpMesh_dst,
        cv::Size frameSize, 
        cv::Size meshGridSize)
    {

        cv::Mat warpMesh_proxy = primeMesh_src.clone();
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
        for(int i = 1; i < warpMesh_proxy.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_proxy.cols - 1; ++j)
            {
                p_tmp = primeMesh_src.at<cv::Point2i>(i, j);

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

                warpMesh_proxy.at<cv::Point2i>(i, j) = {xdist, ydist};
            }
        }
        warpMesh_dst = warpMesh_proxy;
    }

}
