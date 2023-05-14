#include "space_level_aug_pipeline_components_zoo.hpp"
#include "warp_mesh_applicator.hpp"

using namespace augmentation::pipeline_comps::space_level::mesh_distortion::random_move::random_mesh_move;

augmentation::
    pipeline_comps::
        space_level::
            mesh_distortion::
                random_move::
                    random_mesh_move::
                        RandomMeshNodesMoveComponent::
                            RandomMeshNodesMoveComponent(const cv::Mat &prime_mesh, const cv::Mat &warp_mesh)
{
    assert(MeshWarpApplicator::checkValidMesh(prime_mesh));
    assert(MeshWarpApplicator::checkValidMesh(warp_mesh));
}

bool augmentation::pipeline_comps::space_level::mesh_distortion::random_move::random_mesh_move::RandomMeshNodesMoveComponent::apply(const MarkedFrame &src, MarkedFrame &dst) const
{
    return false;
}

std::shared_ptr<RandomMeshNodesMoveComponent> augmentation::
    pipeline_comps::
        space_level::
            mesh_distortion::
                random_move::
                    random_mesh_move::
                        createRandomValidMesh()
{
    return std::shared_ptr<RandomMeshNodesMoveComponent>();
}
