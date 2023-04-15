#ifndef CELL_MOVER_MESH_GRID_NODES_CALLBACK_HPP
#define CELL_MOVER_MESH_GRID_NODES_CALLBACK_HPP


#include <functional>

namespace mesh_nodes_move
{
    class CellMoverMeshGridNodesCallback
    {
        public:
            CellMoverMeshGridNodesCallback(const std::function<double(double)> callback);
            CellMoverMeshGridNodesCallback();
            double operator() (double arg) const; // получение значения функции
            bool is_valid() const;
        private:
            void checkValidCallback();
            enum CellMoverStates
            {
                CALLBACK_INSTALLED,
                SUCCESS_CHECK_VALID_CALLBACK,
                NOT_SUCCESS_CHECK_VALID_CALLBACK
            };
            CellMoverStates mover_state = CALLBACK_INSTALLED;            
            std::function<double(double)> callback_core;
    };
};

#endif //CELL_MOVER_MESH_GRID_NODES_CALLBACK_HPP
