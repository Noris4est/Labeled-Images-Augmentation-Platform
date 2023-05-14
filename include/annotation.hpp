#ifndef ANNOTATION_HPP
#define ANNOTATION_HPP

#include <string>
#include "common_types_project.hpp"
#include "single_line_annotation.hpp"
#include <vector>
#include <map>

namespace annotation
{
    class Annotation
    {
    public:
        Annotation();
        bool readTxt(const std::string &path2srcTxt);
        bool saveTxt(const std::string &path2dstDir, const std::string &filename) const;
        SingleLineAnnotation& operator[](int);
        SingleLineAnnotation operator[](int) const;
        int size() const;
        bool empty() const;
        
        void push_back(const SingleLineAnnotation &annot_s);//добавить объект в аннотацию
        void push_back(const LabledRelBbox &lrbbox);
        void push_back(int class_id, float xc, float yc, float w, float h);

        void putSrcMarkFrame2dstFrame_markRecalculation(
            cv::Size srcFrameSize, 
            cv::Size dstFrameSize, 
            cv::Point tl_corner_dst,
            double intersectionAreaThreshold);

        void clear(); //очистить аннотацию

        // захват в ROI объектов и пересчет их координат (в координатах ROI)
        int captureAndMarkRecalculation(cv::Rect2d ROI, Annotation &resultAnnot, double intersectionAreaTreshold) const;

        // Применить аффинное преобразование к аннотации
        void applyAffineTransform(cv::Mat warpMat, cv::Size origFrameSize, cv::Size warpFrameSize);

        // Применить перспективное преобразование к аннотации
        void applyPerspectiveTransform(cv::Mat warpMat, cv::Size origFrameSize, cv::Size warpFrameSize);

        void CheckValidAndAdaptation();

        Annotation& operator=(const Annotation& other);
    private:
        std::vector<SingleLineAnnotation> data;
        const std::string path2logDir = "../logs/";
        const std::string invalidAnnotLogTxt = "invalid_annotation_log.txt"; // Логируются пути к неисправляемым .txt файлам аннотации
        const std::string notQuiteCorrectAnnotLogTxt = "not_quite_correct_annotation_log.txt"; // Логируются пути к немного некорректным, но автоматически исправляемым .txt файлам аннотации
    };
    bool getMatFromAnnotationVec_keyIsClassId_valueIsVectorSingleLineAnnotation(
        const annotation::Annotation &annot_src, 
        std::map<int, std::vector<annotation::SingleLineAnnotation>> &dst_split_annot_by_class_id);
    void getSetOfClassesInDatasetAnnots(const std::vector<annotation::Annotation> annots, std::set<int> &classes_zoo);
}

#endif //ANNOTATION_HPP
