#include "distortion_mesh_generator_zoo.hpp"
#include "random_processing.hpp"
#include <functional>
#include <math.h>
#include "mesh_grid_nodes_mover.hpp"
#include "cell_mover_mesh_grid_nodes_callback.hpp"
#include <cmath>
#include "math_utils_common.hpp"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


namespace mesh_generator
{
    /*
    Генерация первичной (неискаженной) сетки изображения в разрешении meshGridSize.
    meshGridSize обозначает разрешение в числе полигонов, а не в числе их вершин.
    */
    cv::Mat createPrimeMesh(cv::Size frameSize, cv::Size meshGridSize) 
    {
        int framew = frameSize.width, frameh = frameSize.height;
        float step_i = (float)frameh / meshGridSize.height;
        float step_j = (float)framew / meshGridSize.width;
        int i_cur, j_cur, i_target, j_target;
        // mesh содержит узлы полигонов, поэтому у него размерность w+1, h+1
        cv::Mat primeMesh(cv::Size(meshGridSize.width + 1, meshGridSize.height + 1), CV_32SC2);

        for(int i = 0; i < primeMesh.rows; ++i) //обходим все вершины mesh
        {
            for(int j = 0; j < primeMesh.cols; ++j)
            {
                i_cur = i * step_i; // координаты исходной ноды
                j_cur = j * step_j;
                if(i_cur == frameh)
                    i_cur -= 1;
                if(j_cur == framew)
                    j_cur -= 1;
                primeMesh.at<cv::Point2i>(i, j) = {j_cur, i_cur};
            }
        }
        return primeMesh;
    }    

    namespace various_others
    {
        /*
        Генерирует искаженную сетку изображения, имеющую характерное растяжения в левой части кадра,
        и сжатие в правой части кадра.
        */
        void creatSqrtWarpMeshFromLeft2Right(const cv::Mat &srcMesh, cv::Mat &warpMesh) // meshGridSize - именно полигоны, определяется автоматически
        {
            assert(!srcMesh.empty());
            assert(srcMesh.rows > 0 && srcMesh.cols > 0);
            assert(srcMesh.type() == CV_32SC2); // cv::Point2i
        
            cv::Size meshGridSize = {srcMesh.cols - 1, srcMesh.rows - 1};
            cv::Size frameSize = srcMesh.at<cv::Point2i>(meshGridSize.height, meshGridSize.width);
            frameSize.width += 1;
            frameSize.height += 1;

            // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
            cv::Mat warpMesh_proxy(srcMesh.size(), srcMesh.type());
            cv::Point p_tmp;
            int i_cur, j_cur, i_target, j_target;

            for(int i = 0; i < warpMesh_proxy.rows; ++i) //обходим все вершины mesh
            {
                for(int j = 0; j < warpMesh_proxy.cols; ++j)
                {
                    p_tmp = srcMesh.at<cv::Point2i>(i, j);
                    i_cur = p_tmp.y; // координаты исходной ноды
                    j_cur = p_tmp.x;
                    i_target = i_cur; // координаты итоговой ноды
                    j_target = round(std::sqrt((frameSize.width) * j_cur) );
                    warpMesh_proxy.at<cv::Point2i>(i, j) = {j_target, i_target};
                }
            }
            warpMesh = warpMesh_proxy;
        }

        /*
        Генерирует искаженную сетку изображения, имеющую характерное растяжения в левой верхней части кадра,
        и сжатие в правой нижней части кадра.
        */
        void creatSqrtWarpMeshFromTopLeft2BottomRight(const cv::Mat &srcMesh, cv::Mat &warpMesh) // meshGridSize - именно полигоны
        {
            assert(!srcMesh.empty());
            assert(srcMesh.rows > 0 && srcMesh.cols > 0);
            assert(srcMesh.type() == CV_32SC2); // cv::Point2i
        
            cv::Size meshGridSize = {srcMesh.cols - 1, srcMesh.rows - 1};
            cv::Size frameSize = srcMesh.at<cv::Point2i>(meshGridSize.height, meshGridSize.width);
            frameSize.width += 1;
            frameSize.height +=1;

            // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
            cv::Mat warpMesh_proxy(srcMesh.size(), srcMesh.type());
            cv::Point p_tmp;
            int i_cur, j_cur, i_target, j_target;

            for(int i = 0; i < warpMesh_proxy.rows; ++i) //обходим все вершины mesh
            {
                for(int j = 0; j < warpMesh_proxy.cols; ++j)
                {
                    p_tmp = srcMesh.at<cv::Point2i>(i, j);
                    i_cur = p_tmp.y; // координаты исходной ноды
                    j_cur = p_tmp.x;
                    i_target = round( std::sqrt((frameSize.height) * i_cur) ); // координаты итоговой ноды
                    j_target = round( std::sqrt((frameSize.width) * j_cur) );
                    warpMesh_proxy.at<cv::Point2i>(i, j) = {j_target, i_target};
                }
            }
            warpMesh = warpMesh_proxy;
        }

        /*
        Генерирует искаженную сетку изображения с характерным положительным масштабированием центральных полигонов и сжатием периферийных.
        Имеет угловатый (прямоугольный) характер действия.
        */
        cv::Mat createExpansionWarpMeshFromCenter2BordersSquare(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize) // meshGridSize - именно полигоны
        {
            // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
            cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);

            int y_center = frameSize.height / 2;
            int halfHeight = y_center - 1; // с запасом для безопасности
            int x_center = frameSize.width / 2;
            int halfWidth = x_center - 1; // с запасом

            int delta_max_percent = 10;
            int delta_max_width = frameSize.width * delta_max_percent / 100;
            int delta_max_height = frameSize.height * delta_max_percent / 100;
            
            //local functions zoo
            std::function<float(float)> f_linear = [](float x)
            {
                return 1 - std::abs(x);
            };
            std::function<float(float)> f_cos = [](float x)
            {
                return std::cos(x * M_PI / 2);
            };
            std::function<float(float)> f_sin = [](float x)
            {
                return 1 - std::sin(x * M_PI / 2);
            };
            auto f_select = f_cos;
            std::function<int(int)> d_w = [delta_max_width, x_center, halfWidth, f_select](int x){
                return sgn(x - x_center) * delta_max_width * f_select((float)(x - x_center) / halfWidth);};

            
            std::function<int(int)> d_h = [delta_max_height, y_center, halfHeight, f_select](int y){
                return sgn(y - y_center) * delta_max_height * f_select((float)(y - y_center) / halfHeight);};


            mesh = primeMesh.clone();
            cv::Point2i tmp_p;
            int dh_cur, dw_cur;
            for(int i = 0; i < mesh.rows; ++i)
            {
                for(int j = 0; j < mesh.cols; ++j)
                {
                    tmp_p = mesh.at<cv::Point2i>(i, j);
                    if(i != 0 && i != mesh.rows - 1)
                    {
                        dh_cur = d_h(tmp_p.y);
                        tmp_p.y += dh_cur;
                    }
                    if(j != 0 && j != mesh.cols - 1)
                    {
                        dw_cur = d_w(tmp_p.x);
                        tmp_p.x += dw_cur;
                    }

                    mesh.at<cv::Point2i>(i,j) = tmp_p;
                }
            }
            return primeMesh;
        }

        /*
        Генерирует искаженную сетку изображения с характерным положительным масштабированием центральных полигонов и сжатием периферийных.
        Имеет гладкий (кругловатый) характер действия.
        */
        cv::Mat createExpansionWarpMeshFromCenter2BordersCircle(cv::Mat &mesh, cv::Size frameSize, cv::Size meshGridSize) // meshGridSize - именно полигоны
        {
            // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
            cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);

            int delta_max_percent = 10;
            
            int maxSideLen = std::max(frameSize.width, frameSize.height);

            int minSideLen = std::min(frameSize.width, frameSize.height);
            int maxShiftAmplitude = minSideLen * delta_max_percent / 100.f;

            //local functions zoo Area Definition: 0...1. Value Area: 1...0; 
            std::function<float(float)> f_linear = [](float x)
            {
                return 1 - std::abs(x);
            };
            std::function<float(float)> f_cos = [](float x)
            {
                return std::cos(x * M_PI / 2);
            };
            std::function<float(float)> f_sin = [](float x)
            {
                return 1 - std::sin(x * M_PI / 2);
            };

            auto f_local = f_cos;
            std::function<float(float)> localAmplitude = [f_local](float r)
            {
                if(r > 0 && r < 1)
                {
                    return f_local(r);
                }
                else
                {
                    return 0.f;
                }
            };

            // Функция f(r) расчета амплитуды сдвига в завимивости от удаления от центра изображения
            std::function<float(float)> shiftAmplitude = [maxShiftAmplitude, localAmplitude, minSideLen](float r)
            {
                return maxShiftAmplitude * localAmplitude((float)r/ (minSideLen));
            };

            mesh = primeMesh.clone();
            cv::Point2i center = {frameSize.width / 2, frameSize.height / 2};
            cv::Point2i cur_p;
            float dx_center2cur, dy_center2cur, dist_center2cur, dx_norm, dy_norm, dx_val, dy_val, amplitude;
            for(int i = 0; i < mesh.rows; ++i)
            {
                for(int j = 0; j < mesh.cols; ++j)
                {
                    cur_p = mesh.at<cv::Point2i>(i, j);
                    dx_center2cur = cur_p.x - center.x;
                    dy_center2cur = cur_p.y - center.y;
                    dist_center2cur = std::sqrt(dx_center2cur * dx_center2cur + dy_center2cur * dy_center2cur);
                    dx_norm = dx_center2cur / dist_center2cur;
                    dy_norm = dy_center2cur / dist_center2cur;
                    amplitude = shiftAmplitude(dist_center2cur);
                    dx_val = amplitude * dx_norm;
                    dy_val = amplitude * dy_norm;
                    if(i != 0 && i != mesh.rows - 1)
                        cur_p.y += dy_val;
                    if(j != 0 && j != mesh.cols - 1)
                        cur_p.x += dx_val;

                    mesh.at<cv::Point2i>(i,j) = cur_p;
                }
            }
            return primeMesh;
        }
    }  
}


 