#include "augmentation_pipeline_component_base.hpp"
#include <stdexcept>

bool augmentation::PipelineComponentBase::apply(const MarkedFrame &, MarkedFrame &dst)
{
    throw std::runtime_error("Error: not override method apply!");
}