#ifndef MESH_GRID_NODES_MOVER_HPP
#define MESH_GRID_NODES_MOVER_HPP

#include <functional>
#include <opencv2/opencv.hpp>
#include <vector>
#include "cell_mover_mesh_grid_nodes_callback.hpp"
#include "mesh_grid_nodes_mover_common.hpp"

namespace mesh_nodes_move
{
    struct CallbackDistributionData
    {
        cv::Mat distribution_cell_directions; // 1ch 
        cv::Mat distribution_cell_callbacks_ids; // 1ch
        std::vector<mesh_nodes_move::CellMoverMeshGridNodesCallback> callback_vec;
        CallbackDistributionData clone() const;
    };

    class MeshGridNodesMover
    {
        public:
            MeshGridNodesMover(
                cv::Size workFrameSize, 
                const CallbackDistributionData &callback_data);
            cv::Point apply(cv::Point point) const;
        private:
            CallbackDistributionData callback_data;
            cv::Size workFrameSize;
            cv::Size meshCellSize; // размеры одной ячейки
    };

    namespace generate_callback_mesh
    {
        void gen_sin_callbackMesh_x_direction(cv::Size callbackMeshSize, CallbackDistributionData &callback_data_dst); //callbackMesh != meshGrid !!!
        void gen_sin_callbackMesh_y_direction(cv::Size callbackMeshSize, CallbackDistributionData &callback_data_dst); //callbackMesh != meshGrid !!!
        void gen_wave_callbackMesh(
            mesh_nodes_move::CellMoverMeshGridNodesCallback cell_callback, 
            cv::Size callbackMeshSize, 
            CallbackDistributionData &callback_data_dst, 
            WaveCallbackMeshPropagationAxis axis);
    }
} // namespace mesh_nodes_move

#endif //MESH_GRID_NODES_MOVER_HPP