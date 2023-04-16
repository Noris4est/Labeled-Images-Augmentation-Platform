#ifndef ANNOTATION_PROCESSING_HPP
#define ANNOTATION_PROCESSING_HPP

#include <string>
#include <iostream>
#include "common_types_project.hpp"
#include "string_processing.hpp"
#include "single_line_annotation.hpp"

bool parseAndCheckValidSingleLineAnnotation(
    const std::string &line, 
    int i_line, 
    const std::string &path2Txt, 
    LabledRelBbox &lrbbox,
    bool &adaptationImnpemented);
    
bool parseAndCheckValidSingleLineAnnotation(
    const std::string &line, 
    int i_line, 
    const std::string &path2Txt, 
    annotation::SingleLineAnnotation &annot,
    bool &adaptationImnpemented);

#endif //ANNOTATION_PROCESSING_HPP