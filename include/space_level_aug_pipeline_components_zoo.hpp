#ifndef SPACE_LEVEL_AUG_PIPELINE_COMPONENTS_ZOO_HPP
#define SPACE_LEVEL_AUG_PIPELINE_COMPONENTS_ZOO_HPP

#include "augmentation_pipeline_component_base.hpp"
#include "marked_frame.hpp"

namespace augmentation::pipeline_comps::space_level::mesh_distortion::random_move::random_mesh_move
{
    class RandomMeshNodesMoveComponent : public PipelineComponentBase
    {
        public:
            RandomMeshNodesMoveComponent(const cv::Mat &prime_mesh, const cv::Mat &warp_mesh);
            bool apply(const MarkedFrame &src, MarkedFrame &dst) const override;
        private:
            cv::Mat prime_mesh;
            cv::Mat warp_mesh;
    };

    /*
    Создание случайной искаженной сетки, 
    которая проходит проверку на валидность за total_try_max попыток, 
    возвращаемое значение operation_status = true, если удалось сформировать 
    валидную сетку за выделенное количество попыток
    */
    std::shared_ptr<RandomMeshNodesMoveComponent> createRandomValidMesh();
}
#endif // SPACE_LEVEL_AUG_PIPELINE_COMPONENTS_ZOO_HPP