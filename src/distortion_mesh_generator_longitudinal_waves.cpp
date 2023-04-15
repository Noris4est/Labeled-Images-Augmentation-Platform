#include "distortion_mesh_generator_longitudinal_waves.hpp"
#include <functional>
#include <math.h>
#include "mesh_grid_nodes_mover.hpp"
#include "cell_mover_mesh_grid_nodes_callback.hpp"
#include <cmath>


namespace mesh_generator::longitudinal
    {
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
    }
