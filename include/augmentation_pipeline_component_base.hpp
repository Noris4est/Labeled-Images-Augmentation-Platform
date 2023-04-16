#ifndef AUGMENTATION_PIPELINE_COMPONENT_HPP
#define AUGMENTATION_PIPELINE_COMPONENT_HPP

#include "marked_frame.hpp"

namespace augmentation
{
    class PipelineComponentBase
    {
        public:
            virtual bool apply(const MarkedFrame &, MarkedFrame &dst); // return success status operation apply 
            
        private:

    };
}
#endif //AUGMENTATION_PIPELINE_COMPONENT_HPP