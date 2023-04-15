#ifndef MESH_GRID_NODES_MOVER_COMMON
#define MESH_GRID_NODES_MOVER_COMMON

namespace mesh_nodes_move
{
    enum CellMeshDirection
    {
        LEFT,
        RIGHT,
        UP,
        DOWN
    };
    
    enum WaveCallbackMeshPropagationAxis
    {
        axisX,
        axisY
    }; // Ось распространения 
}

#endif //MESH_GRID_NODES_MOVER_COMMON