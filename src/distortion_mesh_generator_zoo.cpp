#include "distortion_mesh_generator_zoo.hpp"
#include "random_processing.hpp"
#include <functional>
#include <math.h>
#include "mesh_grid_nodes_mover.hpp"
#include "cell_mover_mesh_grid_nodes_callback.hpp"
#include <cmath>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float ipow(float x, int n)
{
    float product = 1;
    for(int i = 0; i < n; ++i)
    {
        product *= x;
    }
    return product;
}

namespace meshGenerator
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

    cv::Mat createRandomWarpMes(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize)
    {
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        float scaleFactor = 0.5;
        int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor * frameSize.height / meshGridSize.height / 2;
        
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1; ++j)
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

    // генерация искаженной сетки, с характерным узором синуса, распространяющегося вдоль x, с изменением координат y сетки
    cv::Mat createTransverseSinWarpMesh_propX_changeY(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor)
    {
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        // int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor * frameSize.height / meshGridSize.height / 2;
        int meshWidthPeriod = frameSize.width / meshGridSize.width;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period = 8 ; // 2 - min по теореме Котельникова
        int sin_period = sin_period_divided_grid_period * meshWidthPeriod;
        double sin_arg = 2 * M_PI / sin_period; 
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                i_target = i_cur + heightAmplitude * std::sin(sin_arg * j_cur); // координаты итоговой ноды
                j_target = j_cur;
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }

    cv::Mat createTransverseSinWarpMesh_propY_changeX(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor)
    {
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        // int heightAmplitude = scaleFactor * frameSize.height / meshGridSize.height / 2;
        // int meshWidthPeriod = frameSize.width / meshGridSize.width;
        int meshHeightPeriod = frameSize.height / meshGridSize.height;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period = 8 ; // 2 - min по теореме Котельникова
        int sin_period = sin_period_divided_grid_period * meshHeightPeriod;
        double sin_arg = 2 * M_PI / sin_period; 
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                j_target = j_cur + widthAmplitude * std::sin(sin_arg * i_cur); // координаты итоговой ноды
                i_target = i_cur;
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }

    cv::Mat createTransverseSinWarpMesh_propXY_changeXY(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, float scaleFactor_X, float scaleFactor_Y)
    {
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp;
        int widthAmplitude = scaleFactor_X * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor_Y * frameSize.height / meshGridSize.height / 2;
        int meshWidthPeriod = frameSize.width / meshGridSize.width;
        int meshHeightPeriod = frameSize.height / meshGridSize.height;
        double twoPI = M_PI * 2;
        int sin_period_divided_grid_period_x = 8 ; // 2 - min по теореме Котельникова
        int sin_period_divided_grid_period_y = 8 ; // 2 - min по теореме Котельникова
        int sin_period_y = sin_period_divided_grid_period_y * meshHeightPeriod;
        int sin_period_x = sin_period_divided_grid_period_x * meshWidthPeriod;
        double sin_arg_prop_x = 2 * M_PI / sin_period_x; 
        double sin_arg_prop_y = 2 * M_PI / sin_period_y; 
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1; ++j)
            {
                p_tmp = primeMesh.at<cv::Point2i>(i, j);
                i_cur = p_tmp.y; // координаты исходной ноды
                j_cur = p_tmp.x;
                j_target = j_cur + widthAmplitude * std::sin(sin_arg_prop_y * i_cur); // координаты итоговой ноды
                i_target = i_cur + heightAmplitude * std::sin(sin_arg_prop_x * j_cur);
                warpMesh.at<cv::Point2i>(i, j) = {j_target, i_target};
            }
        }
        return primeMesh;
    }

    void createLongitudinalWaveSinWarpMeshDirectionX(const cv::Mat &primeMesh, cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize, cv::Size callbackMeshSize)
    {
        warpMesh = primeMesh.clone();

        mesh_nodes_move::CallbackDistributionData callback_data;
        mesh_nodes_move::generate_callback_mesh::gen_sin_callbackMesh_x_direction(callbackMeshSize, callback_data);
        mesh_nodes_move::MeshGridNodesMover mesh_mover(frameSize, callback_data);

        int i_cur, j_cur, i_target, j_target;
        cv::Point p_tmp_src, p_tmp_dst;
        float scaleFactor = 0.5;
        int widthAmplitude = scaleFactor * frameSize.width / meshGridSize.width / 2;
        int heightAmplitude = scaleFactor * frameSize.height / meshGridSize.height / 2;
        
        for(int i = 1; i < warpMesh.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh.cols - 1; ++j)
            {
                p_tmp_src = primeMesh.at<cv::Point2i>(i, j);
                p_tmp_dst = mesh_mover.apply(p_tmp_src);
                warpMesh.at<cv::Point2i>(i, j) = p_tmp_dst;
            }
        }
    }

    void createLongitudinalWaveSinWarpMeshDirectionY(const cv::Mat &primeMesh_src, cv::Mat &warpMesh_dst, cv::Size frameSize_src, cv::Size meshGridSize_src, cv::Size callbackMeshSize_src)
    {
        warpMesh_dst = primeMesh_src.clone();

        mesh_nodes_move::CallbackDistributionData callback_data;
        mesh_nodes_move::generate_callback_mesh::gen_sin_callbackMesh_y_direction(callbackMeshSize_src, callback_data);
        mesh_nodes_move::MeshGridNodesMover mesh_mover(frameSize_src, callback_data);

        cv::Point p_tmp_src, p_tmp_dst;
        
        for(int i = 1; i < warpMesh_dst.rows - 1; ++i) //обходим все вершины mesh
        {
            for(int j = 1; j < warpMesh_dst.cols - 1; ++j)
            {
                p_tmp_src = primeMesh_src.at<cv::Point2i>(i, j);
                p_tmp_dst = mesh_mover.apply(p_tmp_src);
                warpMesh_dst.at<cv::Point2i>(i, j) = p_tmp_dst;
            }
        }
    }

    void createLongitudinalWaveBaseCallbackWarpMesh(std::function<double(double)> base_callback, const cv::Mat &primeMesh_src, cv::Mat &warpMesh_dst, cv::Size frameSize_src, cv::Size meshGridSize_src, cv::Size callbackMeshSize_src, mesh_nodes_move::WaveCallbackMeshPropagationAxis axis, bool change_position_left_and_right_border_mesh_nodes, bool change_position_bottom_and_top_border_mesh_nodes)
    {
        cv::Mat warpMesh_dst_proxy = primeMesh_src.clone();

        mesh_nodes_move::CellMoverMeshGridNodesCallback callback(base_callback);
        mesh_nodes_move::CallbackDistributionData callback_data;
        mesh_nodes_move::generate_callback_mesh::gen_wave_callbackMesh(callback, callbackMeshSize_src, callback_data, axis);
        mesh_nodes_move::MeshGridNodesMover mesh_mover(frameSize_src, callback_data);

        cv::Point p_tmp_src, p_tmp_dst;
        
        int i_begin, j_begin, i_end, j_end;
        if(change_position_left_and_right_border_mesh_nodes)
        {
            j_begin = 0;
            j_end = warpMesh_dst_proxy.cols;
        }
        else
        {
            j_begin = 1;
            j_end = warpMesh_dst_proxy.cols - 1;
        };

        if(change_position_bottom_and_top_border_mesh_nodes)
        {
            i_begin = 0;
            i_end = warpMesh_dst_proxy.rows; // как с итераторами (последний указывает на следующий за последним в контейнере)
        }
        else
        {
            i_begin = 1;
            i_end = warpMesh_dst_proxy.rows - 1;
        }
        
        for(int i = i_begin; i < i_end; ++i) //обходим все вершины mesh
        {
            for(int j = j_begin; j < j_end; ++j)
            {
                p_tmp_src = primeMesh_src.at<cv::Point2i>(i, j);
                p_tmp_dst = mesh_mover.apply(p_tmp_src);
                warpMesh_dst_proxy.at<cv::Point2i>(i, j) = p_tmp_dst;
            }
        }
        warpMesh_dst = warpMesh_dst_proxy;
    }

    void createLongitudinalWaveSinWarpMeshDistortion(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Size callbackMeshSize_src, 
        mesh_nodes_move::WaveCallbackMeshPropagationAxis axis, 
        bool change_position_left_and_right_border_mesh_nodes, 
        bool change_position_bottom_and_top_border_mesh_nodes)
    {
        std::function<double(double)> base_callback = [](double arg)
        {
            return std::sin(M_PI_2 * arg);
        }; // f(0) = 0, f(1) = 1 df/dx > 0 x in [0,1]!
        createLongitudinalWaveBaseCallbackWarpMesh(
            base_callback, 
            primeMesh_src, 
            warpMesh_dst, 
            frameSize_src, 
            meshGridSize_src, 
            callbackMeshSize_src, 
            axis, 
            change_position_left_and_right_border_mesh_nodes, 
            change_position_bottom_and_top_border_mesh_nodes);
    }

    void createLongitudinalWaveGammaWarpMeshDistortion(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Size callbackMeshSize_src, 
        double gamma_coefficient,
        mesh_nodes_move::WaveCallbackMeshPropagationAxis axis,
        bool change_position_left_and_right_border_mesh_nodes,
        bool change_position_bottom_and_top_border_mesh_nodes)
    {
        cv::Mat warpMesh_dst_proxy = primeMesh_src.clone();
        // warpMesh_dst = primeMesh_src.clone();

        std::function<double(double)> callback_prime = [gamma_coefficient](double arg)
        {
            return std::pow(arg, gamma_coefficient);
        };
        mesh_nodes_move::CellMoverMeshGridNodesCallback callback(callback_prime);
        mesh_nodes_move::CallbackDistributionData callback_data;
        mesh_nodes_move::generate_callback_mesh::gen_wave_callbackMesh(callback, callbackMeshSize_src, callback_data, axis);
        mesh_nodes_move::MeshGridNodesMover mesh_mover(frameSize_src, callback_data);

        cv::Point p_tmp_src, p_tmp_dst;
        
        int i_begin, j_begin, i_end, j_end;
        if(change_position_left_and_right_border_mesh_nodes)
        {
            j_begin = 0;
            j_end = warpMesh_dst_proxy.cols;
        }
        else
        {
            j_begin = 1;
            j_end = warpMesh_dst_proxy.cols - 1;
        };

        if(change_position_bottom_and_top_border_mesh_nodes)
        {
            i_begin = 0;
            i_end = warpMesh_dst_proxy.rows; // как с итераторами (последний указывает на следующий за последним в контейнере)
        }
        else
        {
            i_begin = 1;
            i_end = warpMesh_dst_proxy.rows - 1;
        }
        
        for(int i = i_begin; i < i_end; ++i) //обходим все вершины mesh
        {
            for(int j = j_begin; j < j_end; ++j)
            {
                p_tmp_src = primeMesh_src.at<cv::Point2i>(i, j);
                p_tmp_dst = mesh_mover.apply(p_tmp_src);
                warpMesh_dst_proxy.at<cv::Point2i>(i, j) = p_tmp_dst;
            }
        }
        warpMesh_dst = warpMesh_dst_proxy;
    }

    void createLongitudinalWaveSinFromSourcePointConcentric(
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Point sourcePoint,
        double halfPeriodOfWaveDividedByMeshCellDiag)
    {
        std::function<double(double)> callback_base = [](double arg)
        {
            return std::sin(M_PI_2 * arg);
        };
        createLongitudinalWaveBaseCallbackFromSourcePointConcentric(
            callback_base,
            primeMesh_src,
            warpMesh_dst,
            frameSize_src,
            meshGridSize_src,
            sourcePoint,
            halfPeriodOfWaveDividedByMeshCellDiag);
    } // createLongitudinalWaveSinFromSourcePoint

    void createLongitudinalWaveBaseCallbackFromSourcePointConcentric(
        std::function<double(double)> base_callback, 
        const cv::Mat &primeMesh_src, 
        cv::Mat &warpMesh_dst, 
        cv::Size frameSize_src, 
        cv::Size meshGridSize_src, 
        cv::Point sourcePoint, 
        double halfPeriodOfWaveDividedByMeshCellDiag)
    {
        assert(!primeMesh_src.empty());
        assert(primeMesh_src.rows !=0 && primeMesh_src.cols != 0);
        assert(frameSize_src.width != 0 && frameSize_src.height !=0);

        cv::Point2i primeMesh_br = primeMesh_src.at<cv::Point2i>(primeMesh_src.rows - 1, primeMesh_src.cols - 1);
        assert(primeMesh_br.x + 1 == frameSize_src.width && primeMesh_br.y + 1 == frameSize_src.height);

        assert(sourcePoint.x >= 0 && sourcePoint.x < frameSize_src.width && sourcePoint.y > 0 && sourcePoint.y < frameSize_src.height);

        std::function<double(double,double)> calc_hypotenuse = [](double a, double b)
        {
            return std::sqrt(a*a + b*b);
        };

        std::function<double(double)> callback_mover = [base_callback](double arg)
        {
            return base_callback(arg);
            // return arg;
        };

        std::function<double(double)> inv_callback_mover = [callback_mover](double arg)
        {   
            return 1 - callback_mover(1 - arg);
        };


        cv::Mat warpMesh_dst_proxy = primeMesh_src.clone();

        double cell_width, cell_height;
        cell_width = frameSize_src.width / meshGridSize_src.width;
        cell_height = frameSize_src.height / meshGridSize_src.height;
        double cell_diag = std::sqrt(cell_width*cell_width + cell_height*cell_height);
        double waveHalfPeriod = halfPeriodOfWaveDividedByMeshCellDiag*cell_diag;
        double wavePeriod = waveHalfPeriod * 2;
        int i_begin = 1, j_begin = 1, i_end = meshGridSize_src.height, j_end = meshGridSize_src.width;
        cv::Point point_src, point_dst;
        int vec_source2node_dx, vec_source2node_dy;
        double len_vec_source2node;
        double vec_s2n_dl_abs; // абсолютное приращение длины вектора в зависимости от номера "кольца", в которое попала вершина сетки географически; s2n = source2node
        double vec_s2n_dl_rel; // Оно же будет равняться приращению для dx и dy
        double division_len_vec_s2n_2_wavePeriod, division_len_vec_s2n_2_waveHalfPeriod; // частное между делимым len_vec_source2node и делителем wavePeriod
        double frac_part_div, frac_part_div_for_half_period;
        double int_part_div, int_part_div_for_half_period; // т.к. у std::modf нет переопределение на целочисленный второй аргумент
        double abs_coordinate_node_in_ring;
        double rel_coordinate_node_in_ring; // относительный радиус
        double start_radius_from_source_current_ring; // начальный радиус текущего кольца (в котором находится node)

        double warp_rel_coordinate_node_in_ring, warp_abs_coordinate_node_in_ring;

        double delta_rel_coordinate_in_ring; //delta radius
        double delta_abs_coordinate_in_ring; 

        int vec_source2node_dx_dst, vec_source2node_dy_dst;

        for(int i = i_begin; i < i_end; ++i) //обходим все вершины mesh
        {
            for(int j = j_begin; j < j_end; ++j)
            {
                point_src = primeMesh_src.at<cv::Point2i>(i, j);

                // processing point_src -> point_dst
                vec_source2node_dx = point_src.x - sourcePoint.x;
                vec_source2node_dy = point_src.y - sourcePoint.y;

                len_vec_source2node = calc_hypotenuse(vec_source2node_dx, vec_source2node_dy);
                
                division_len_vec_s2n_2_wavePeriod = len_vec_source2node / wavePeriod;
                frac_part_div = std::modf(division_len_vec_s2n_2_wavePeriod, &int_part_div);
                
                division_len_vec_s2n_2_waveHalfPeriod = len_vec_source2node / waveHalfPeriod;
                frac_part_div_for_half_period = std::modf(division_len_vec_s2n_2_waveHalfPeriod, &int_part_div_for_half_period);
                
                // start_radius_from_source_current_ring = int_part_div_for_half_period * waveHalfPeriod;
                
                // abs_coordinate_node_in_ring = len_vec_source2node - start_radius_from_source_current_ring;
                // rel_coordinate_node_in_ring = abs_coordinate_node_in_ring / waveHalfPeriod;
                rel_coordinate_node_in_ring = frac_part_div_for_half_period;
                if(frac_part_div < 0.5)
                {   
                    warp_rel_coordinate_node_in_ring = callback_mover(rel_coordinate_node_in_ring);
                    //direct callback
                    
                }
                else
                {
                    warp_rel_coordinate_node_in_ring = inv_callback_mover(rel_coordinate_node_in_ring);
                    //inverse callback
                }
                delta_rel_coordinate_in_ring = warp_rel_coordinate_node_in_ring - rel_coordinate_node_in_ring;
                delta_abs_coordinate_in_ring = delta_rel_coordinate_in_ring * waveHalfPeriod;
                vec_s2n_dl_abs = delta_abs_coordinate_in_ring;
                vec_s2n_dl_rel = vec_s2n_dl_abs / len_vec_source2node ;
                vec_source2node_dx_dst = vec_source2node_dx * (1 + vec_s2n_dl_rel);
                vec_source2node_dy_dst = vec_source2node_dy * (1 + vec_s2n_dl_rel);
                point_dst.x = vec_source2node_dx_dst + sourcePoint.x;
                point_dst.y = vec_source2node_dy_dst + sourcePoint.y;
                warpMesh_dst_proxy.at<cv::Point2i>(i, j) = point_dst;
            }
        }
        warpMesh_dst = warpMesh_dst_proxy;
    }

    /*
    Генерирует искаженную сетку изображения, имеющую характерное растяжения в левой части кадра,
    и сжатие в правой части кадра.
    */
    cv::Mat creatSqrtWarpMeshFromLeft2Right(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize) // meshGridSize - именно полигоны
    {
        // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        cv::Point p_tmp;
        int i_cur, j_cur, i_target, j_target;
        for(int i = 0; i < warpMesh.rows; ++i) //обходим все вершины mesh
        {
            for(int j = 0; j < warpMesh.cols; ++j)
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

    /*
    Генерирует искаженную сетку изображения, имеющую характерное растяжения в левой верхней части кадра,
    и сжатие в правой нижней части кадра.
    */
    cv::Mat creatSqrtWarpMeshFromTopLeft2BottomRight(cv::Mat &warpMesh, cv::Size frameSize, cv::Size meshGridSize) // meshGridSize - именно полигоны
    {
        // mesh содержит узлы полиномов, поэтому у него размерность w+1, h+1
        cv::Mat primeMesh = createPrimeMesh(frameSize, meshGridSize);
        warpMesh = primeMesh.clone();
        cv::Point p_tmp;
        int i_cur, j_cur, i_target, j_target;
        for(int i = 0; i < warpMesh.rows; ++i) //обходим все вершины mesh
        {
            for(int j = 0; j < warpMesh.cols; ++j)
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
            for(int j = 1; j < warpMesh.cols - 1; ++j)
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

 