#include "cell_mover_mesh_grid_nodes_callback.hpp"

mesh_nodes_move::CellMoverMeshGridNodesCallback::CellMoverMeshGridNodesCallback(const std::function<double(double)> callback)
{
    this->callback_core = callback;
    mover_state = CellMoverStates::CALLBACK_INSTALLED;
    checkValidCallback();
}

mesh_nodes_move::CellMoverMeshGridNodesCallback::CellMoverMeshGridNodesCallback()
{
    mover_state = NOT_SUCCESS_CHECK_VALID_CALLBACK;
}

double mesh_nodes_move::CellMoverMeshGridNodesCallback::operator()(double arg) const
{
    if(mover_state == CellMoverStates::SUCCESS_CHECK_VALID_CALLBACK)
    {
        return callback_core(arg);
    }
    else
    {
        throw std::runtime_error("Cast callback() when mover_state != CellMoverStates::SUCCESS_CHECK_VALID_CALLBACK");
    }
}

bool mesh_nodes_move::CellMoverMeshGridNodesCallback::is_valid() const
{
    return mover_state == CellMoverStates::SUCCESS_CHECK_VALID_CALLBACK;
}

void mesh_nodes_move::CellMoverMeshGridNodesCallback::checkValidCallback()
{
    if(mover_state != CALLBACK_INSTALLED)
    {
        throw std::runtime_error("Error: incorrect call checkValidCallback()");
    }
    bool callback_is_valid = true;
    double tolerance = 0.01;
    // f(0) = 0, f(1) = 1
    callback_is_valid &=
        callback_core(0) < tolerance && 
        callback_core(0) > -tolerance &&
        callback_core(1) > 1 - tolerance &&
        callback_core(1) < 1 + tolerance;

    if(callback_is_valid)
    {
        mover_state = CellMoverStates::SUCCESS_CHECK_VALID_CALLBACK;
    }
    else
    {
        mover_state = CellMoverStates::NOT_SUCCESS_CHECK_VALID_CALLBACK;
    }
}
