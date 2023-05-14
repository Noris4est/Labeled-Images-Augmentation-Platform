#include "augmentation_pipeline_component_base.hpp"
#include <stdexcept>

bool augmentation::PipelineComponentBase::apply(const MarkedFrame &src, MarkedFrame &dst) const
{
    return false;
}
