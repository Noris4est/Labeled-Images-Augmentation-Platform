#include "mesh_grid_nodes_mover.hpp"
#include <opencv2/opencv.hpp>
#include <functional>
#include "cell_mover_mesh_grid_nodes_callback.hpp"
#include "mesh_grid_nodes_mover_common.hpp"

void mesh_nodes_move::generate_callback_mesh::gen_sin_callbackMesh_x_direction(
    cv::Size meshGridSize, 
    CallbackDistributionData &callback_data)
{
    int rows = meshGridSize.height;
    int cols = meshGridSize.width;
    assert(rows > 0 && cols > 0);
    callback_data.distribution_cell_callbacks_ids = cv::Mat(meshGridSize, CV_8UC1);
    callback_data.distribution_cell_directions = cv::Mat(meshGridSize, CV_8UC1);
    std::function<double(double)> sin_callback = [](double arg)
    {
        return std::sin(M_PI_2 * arg);
    };

    mesh_nodes_move::CellMoverMeshGridNodesCallback cellCallback(sin_callback);
    if(!cellCallback.is_valid())
    {
        throw std::runtime_error("cellCallback != is_valid!");
    }
    callback_data.callback_vec.clear();
    callback_data.callback_vec.push_back(cellCallback);

    CellMeshDirection distr_tmp;
    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            if(j%2)
            {
                distr_tmp = CellMeshDirection::LEFT;
            }
            else
            {
                distr_tmp = CellMeshDirection::RIGHT;
            }
            callback_data.distribution_cell_directions.at<uint8_t>(i, j) = static_cast<uint8_t>(distr_tmp);
            callback_data.distribution_cell_callbacks_ids.at<uint8_t>(i, j) = static_cast<uint8_t>(0);
        }
    }
}

void mesh_nodes_move::generate_callback_mesh::gen_sin_callbackMesh_y_direction(cv::Size callbackMeshSize, CallbackDistributionData &callback_data_dst)
{
    int rows = callbackMeshSize.height;
    int cols = callbackMeshSize.width;
    assert(rows > 0 && cols > 0);
    callback_data_dst.distribution_cell_callbacks_ids = cv::Mat(callbackMeshSize, CV_8UC1);
    callback_data_dst.distribution_cell_directions = cv::Mat(callbackMeshSize, CV_8UC1);
    std::function<double(double)> sin_callback = [](double arg)
    {
        return std::sin(M_PI_2 * arg);
    };

    mesh_nodes_move::CellMoverMeshGridNodesCallback cellCallback(sin_callback);
    if(!cellCallback.is_valid())
    {
        throw std::runtime_error("cellCallback != is_valid!");
    }
    callback_data_dst.callback_vec.clear();
    callback_data_dst.callback_vec.push_back(cellCallback);

    CellMeshDirection distr_tmp;
    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            if(i%2)
            {
                distr_tmp = CellMeshDirection::DOWN;
            }
            else
            {
                distr_tmp = CellMeshDirection::UP;
            }
            callback_data_dst.distribution_cell_directions.at<uint8_t>(i, j) = static_cast<uint8_t>(distr_tmp);
            callback_data_dst.distribution_cell_callbacks_ids.at<uint8_t>(i, j) = static_cast<uint8_t>(0);
        }
    }
}

void mesh_nodes_move::generate_callback_mesh::gen_wave_callbackMesh(
    mesh_nodes_move::CellMoverMeshGridNodesCallback cell_callback, 
    cv::Size callbackMeshSize, 
    CallbackDistributionData &callback_data_dst, 
    WaveCallbackMeshPropagationAxis axis)
{
    int rows = callbackMeshSize.height;
    int cols = callbackMeshSize.width;
    assert(rows > 0 && cols > 0);
    callback_data_dst.distribution_cell_callbacks_ids = cv::Mat(callbackMeshSize, CV_8UC1);
    callback_data_dst.distribution_cell_directions = cv::Mat(callbackMeshSize, CV_8UC1);
    if(!cell_callback.is_valid())
    {
        throw std::runtime_error("Error: invalid cell_callback");
    }
    CellMeshDirection plus_dir, minus_dir; // условные обозначения (паттерн стратегия)

    // Проверка четности parity checker - паттерн "стратегия"
    std::function<bool(int, int)> parity_checker;

    if(axis == WaveCallbackMeshPropagationAxis::axisX)
    {
        plus_dir = CellMeshDirection::RIGHT;
        minus_dir = CellMeshDirection::LEFT;
        parity_checker = [](int i, int j)
        {
            return j%2;
        };
    }
    else if(axis == WaveCallbackMeshPropagationAxis::axisY)
    {
        plus_dir = CellMeshDirection::UP;
        minus_dir = CellMeshDirection::DOWN;
        parity_checker = [](int i, int j)
        {
            return i%2;
        };
    }
    else
    {
        throw std::runtime_error("Error: not supported axis in enum WaveCallbackMeshPropagationAxis");
    }

    callback_data_dst.callback_vec.clear();
    callback_data_dst.callback_vec.push_back(cell_callback);

    CellMeshDirection distr_tmp;

    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            if(parity_checker(i,j))
            {
                distr_tmp = minus_dir;
            }
            else
            {
                distr_tmp = plus_dir;
            }
            callback_data_dst.distribution_cell_directions.at<uint8_t>(i, j) = static_cast<uint8_t>(distr_tmp);
            callback_data_dst.distribution_cell_callbacks_ids.at<uint8_t>(i, j) = static_cast<uint8_t>(0);
        }
    }

}


mesh_nodes_move::MeshGridNodesMover::MeshGridNodesMover(cv::Size workFrameSize, const CallbackDistributionData &callback_data)
{
    assert(callback_data.distribution_cell_callbacks_ids.size() == callback_data.distribution_cell_directions.size());
    cv::Size callbackMeshSize = callback_data.distribution_cell_callbacks_ids.size();
    this->callback_data = callback_data.clone();
    this->workFrameSize = workFrameSize;
    meshCellSize.width = workFrameSize.width / callbackMeshSize.width;
    meshCellSize.height = workFrameSize.height / callbackMeshSize.height;
}

cv::Point mesh_nodes_move::MeshGridNodesMover::apply(cv::Point point) const
{
    int i_mesh = point.y / meshCellSize.height; // целая часть от деления
    int j_mesh = point.x / meshCellSize.width; 
    
    int callback_id = callback_data.distribution_cell_callbacks_ids.at<uint8_t>(i_mesh, j_mesh);
    CellMeshDirection cell_direction = static_cast<CellMeshDirection>(callback_data.distribution_cell_directions.at<uint8_t>(i_mesh, j_mesh));
    cv::Point tl_corner_cell = {j_mesh * meshCellSize.width, i_mesh * meshCellSize.height};
    cv::Point src_point_in_cell_coords = {point.x - tl_corner_cell.x, point.y - tl_corner_cell.y};
    
    double x_rel_cell_src = static_cast<double>(src_point_in_cell_coords.x) / meshCellSize.width;
    double y_rel_cell_src = static_cast<double>(src_point_in_cell_coords.y) / meshCellSize.height;

    double x_rel_cell_dst = x_rel_cell_src, y_rel_cell_dst = y_rel_cell_src;

    auto callback = callback_data.callback_vec[callback_id];
    switch (cell_direction)
    {
    case CellMeshDirection::UP:
        {
            y_rel_cell_dst = callback(y_rel_cell_src);
        }
        break;
    case CellMeshDirection::DOWN:
        {
            y_rel_cell_dst = 1 - callback(1 - y_rel_cell_src);
        }
        break;
    case CellMeshDirection::RIGHT:
        {  
            x_rel_cell_dst = callback(x_rel_cell_src);
        }
        break;
    case CellMeshDirection::LEFT:
        {
            x_rel_cell_dst = 1 - callback(1 - x_rel_cell_src);
        }
        break;
    default:
        {
            throw std::runtime_error("Error: not support Cell Mesh Direction");
        }
        break;
    }

    int x_abs_cell_dst = round(x_rel_cell_dst * meshCellSize.width);
    int y_abs_cell_dst = round(y_rel_cell_dst * meshCellSize.height);
    int x_abs_frame_dst = tl_corner_cell.x + x_abs_cell_dst;
    int y_abs_frame_dst = tl_corner_cell.y + y_abs_cell_dst;
    return {x_abs_frame_dst, y_abs_frame_dst};
}

mesh_nodes_move::CallbackDistributionData mesh_nodes_move::CallbackDistributionData::clone() const
{
    CallbackDistributionData tmp;
    tmp.distribution_cell_callbacks_ids = this->distribution_cell_callbacks_ids.clone();
    tmp.distribution_cell_directions = this->distribution_cell_directions.clone();
    tmp.callback_vec = this->callback_vec;
    return tmp;
}
