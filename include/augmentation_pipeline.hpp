#ifndef AUGMENTATION_PIPELINE_HPP
#define AUGMENTATION_PIPELINE_HPP

#include "marked_frame.hpp"
#include <vector>
#include <memory>
#include "augmentation_pipeline_component_base.hpp"

namespace augmentation
{
    class AugmentationPipeline
    {
        public:
            void apply_forward(const MarkedFrame &src, MarkedFrame &dst) const; // прямой проход
            void apply_opposite(const MarkedFrame &src, MarkedFrame &dst) const; // обратный проход
            size_t size() const;
            
            void push_back(std::shared_ptr<augmentation::PipelineComponentBase> component); // расширение пайплайна
            void push_front(std::shared_ptr<augmentation::PipelineComponentBase> component);
            std::shared_ptr<augmentation::PipelineComponentBase> pop_back();
            std::shared_ptr<augmentation::PipelineComponentBase> pop_front();
        private:
            std::vector<std::shared_ptr<augmentation::PipelineComponentBase>> pipeline;
    };
}
#endif // AUGMENTATION_PIPELINE_HPP