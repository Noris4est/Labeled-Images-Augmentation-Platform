#include "annotation.hpp"

#include <vector>
#include <fstream>
#include "single_line_annotation.hpp"
#include "annotation_processing.hpp"
#include "string_processing.hpp"
#include "rect_processing.hpp"
#include "affine_processing.hpp"
#include "path_processing.hpp"

using namespace annotation;

Annotation::Annotation()
{
    if(!path_processing::isDirExist(path2logDir))
    {
        path_processing::makePath(path2logDir);
    }

    if (!path_processing::isRegularFileExist(path2logDir + "/" + invalidAnnotLogTxt))
    {
        std::ofstream invalidAnnotLogFile; 
        invalidAnnotLogFile.open(path2logDir + "/" +invalidAnnotLogTxt);
        invalidAnnotLogFile.close();
    }
    if (!path_processing::isRegularFileExist(path2logDir + "/" + notQuiteCorrectAnnotLogTxt))
    {
        std::ofstream adaptationImnpementedLogFile; 
        adaptationImnpementedLogFile.open(path2logDir + "/" + notQuiteCorrectAnnotLogTxt);
        adaptationImnpementedLogFile.close();  
    }
}

bool Annotation::readTxt(const std::string &path2srcTxt)
{
    data.clear();
    data.reserve(5);
    std::ifstream ifs(path2srcTxt); // открываем одноименный с изображением текстовый файл в srcDir
    if(!ifs.is_open())
    {
        //std::cout << "Warning: file : " << path2srcTxt << " is not exits!" << std::endl;
        return false;
    }
    std::string line;
    int line_counter = 0;
    annotation::SingleLineAnnotation annot_s;
    bool adaptationImnpemented = false, adaptationImnpemented_tmp;
    bool invalidAnnotStatus = false;
    while (getline(ifs, line))
    {
        string_processing::strip(line, ' ');
        if(line == "")
            continue;
        if(!parseAndCheckValidSingleLineAnnotation(line, line_counter, path2srcTxt, annot_s, adaptationImnpemented_tmp))
        {
            invalidAnnotStatus = true;
            continue;
        }
        if(adaptationImnpemented_tmp)
            adaptationImnpemented = true;
        data.push_back(annot_s);
        ++line_counter;
    }
    ifs.close();
    
    if(invalidAnnotStatus)
    {
        std::ofstream invalidAnnotLogFile; 
        invalidAnnotLogFile.open(path2logDir + "/" +invalidAnnotLogTxt, std::ios::app);
        invalidAnnotLogFile << path2srcTxt << "\n";
        invalidAnnotLogFile.close();
        return false;
        //throw std::runtime_error("Error: invalid annotation! path=\"" + path2srcTxt + "\"");
    }

    if(adaptationImnpemented)
    {
        std::ofstream adaptationImnpementedLogFile; 
        adaptationImnpementedLogFile.open(path2logDir + "/" + notQuiteCorrectAnnotLogTxt, std::ios::app);
        adaptationImnpementedLogFile << path2srcTxt << "\n";
        adaptationImnpementedLogFile.close();  
    }

    return true;
} // -- END readTxt

bool Annotation::saveTxt(const std::string &path2dstDir, const std::string &filename) const
{
    if(!path_processing::isDirExist(path2dstDir))
    {
        std::cout << "Warning: directory : " << path2dstDir << "is not exits!" << std::endl;
        return false;
    }
    std::ofstream outfile; 
    std::string filepath = path2dstDir + std::string("/") + filename + std::string(".txt");
    outfile.open(filepath, std::ios::trunc);
    for(int i = 0; i < data.size(); i++)
    {
        outfile << data[i].to_str() << "\n";
    }
    outfile.close();
    return true;
} // -- END saveTxt

SingleLineAnnotation& Annotation::operator[](int index)
{
    if (index >= data.size() || index < 0)
    {
        throw std::runtime_error("index out of range");
    }
    return data[index];
} // -- END operator[]

SingleLineAnnotation annotation::Annotation::operator[](int index) const
{
    if (index >= data.size() || index < 0)
    {
        throw std::runtime_error("index out of range");
    }
    return data[index];
}

int Annotation::size() const
{
    return data.size();
} // -- END size

bool Annotation::empty() const
{
    return (data.size() == 0);
} // -- END empty

void Annotation::push_back(const SingleLineAnnotation &annot_s)
{
    data.push_back(annot_s);
} // -- END push_back

void Annotation::push_back(const LabledRelBbox &lrbbox)
{
    SingleLineAnnotation annot_s(lrbbox);
    data.push_back(annot_s);
} // -- END push_back

void annotation::Annotation::push_back(int class_id, float xc, float yc, float w, float h)
{
    SingleLineAnnotation current(class_id, xc, yc, w, h);
    data.push_back(current);
} // -- END push_back

void Annotation::clear()
{
    data.clear();
} // -- END clear

/*
    Захват объектов рамкой ROI в относительных единицах, возвращает
    количество захваченных объектов
*/
int Annotation::captureAndMarkRecalculation(
    cv::Rect2d ROI,
    Annotation &resultAnnot,
    double intersectionAreaThreshold) const
{
    resultAnnot.clear();
    resultAnnot.data.reserve(this->size());
    int counter = 0;
    LabledRelBbox lrbbox_tmp;
    cv::Rect2d bbox;
    for(auto &annot_s : data)
    {
        lrbbox_tmp = annot_s.to_lrbbox();
        bbox = lrbbox_tmp.bbox;
        if(calcIntersectionAreaPart(ROI, bbox) >= intersectionAreaThreshold)
        {
            lrbbox_tmp.bbox = catchBboxUsingROI(ROI, bbox);
            if(!lrbbox_tmp.checkValid())
                throw std::runtime_error("Error: incorrect lrbbox");
            resultAnnot.push_back(lrbbox_tmp);
            ++counter;
        }
    }
    resultAnnot.CheckValidAndAdaptation();
    return counter;
} // -- END captureAndMarkRecalculation

void Annotation::applyAffineTransform(cv::Mat warpMat, cv::Size origFrameSize, cv::Size warpFrameSize)
{
    /*
    Преобразование разметок из относительных в абсолютные, 
    применение трансформации, 
    преобразование из абсолютных к относительным координатам
    */
    cv::Rect2d bbox_rel_org, bbox_rel_new;
    cv::RotatedRect bbox_rot;
    cv::Rect2i bbox_abs_org, bbox_abs_new;

    for (auto &annot_s : data)
    {
        bbox_rel_org = annot_s.get_rect();
        bbox_abs_org = getAbsRectFromRel(bbox_rel_org, origFrameSize);
        bbox_rot = warpAffine2Rect(bbox_abs_org, warpMat);
        bbox_abs_new = bbox_rot.boundingRect();
        bbox_rel_new = getRelRectFromAbs(bbox_abs_new, warpFrameSize);
        annot_s.set_rect(bbox_rel_new);
    }
    CheckValidAndAdaptation();
} // -- END applyAffineTransform

void Annotation::applyPerspectiveTransform(cv::Mat warpMat, cv::Size origFrameSize, cv::Size warpFrameSize)
{
    /*
    Преобразование разметок из относительных в абсолютные, 
    применение трансформации, 
    преобразование из абсолютных к относительным координатам
    */
    cv::Rect2d bbox_rel_org, bbox_rel_new;
    cv::Rect2i bbox_abs_org, bbox_abs_new;

    for (auto &annot_s : data)
    {
        bbox_rel_org = annot_s.get_rect();
        bbox_abs_org = getAbsRectFromRel(bbox_rel_org, origFrameSize);
        rect_processing::applyPerspectiveAndGetBounding(bbox_abs_org, bbox_abs_new, warpMat);
        bbox_rel_new = getRelRectFromAbs(bbox_abs_new, warpFrameSize);
        annot_s.set_rect(bbox_rel_new);
    }

    CheckValidAndAdaptation();
} // -- END applyPerspectiveTransform

void Annotation::CheckValidAndAdaptation()
{
    for(auto &slannot : data)
    {
        slannot.CheckValidAndAdaptation();
    }   
}

Annotation& Annotation::operator=(const Annotation& other)
{
    if(this != &other)
    {
        data = other.data;
    }
    return *this;
}

bool annotation::getMatFromAnnotationVec_keyIsClassId_valueIsVectorSingleLineAnnotation(
    const annotation::Annotation &annot_src, 
    std::map<int, std::vector<annotation::SingleLineAnnotation>> &dst_split_annot_by_class_id)
{
    annotation::SingleLineAnnotation slannot;
    dst_split_annot_by_class_id.clear();
    int class_id_tmp;
    for(int i = 0; i < annot_src.size(); ++i)
    {
        slannot = annot_src[i];
        class_id_tmp = slannot.get_class_id();
        if(dst_split_annot_by_class_id.find(class_id_tmp) != dst_split_annot_by_class_id.end())
        {
            dst_split_annot_by_class_id[class_id_tmp].push_back(slannot);
        }
        else
        {
            dst_split_annot_by_class_id[class_id_tmp] = {slannot};
        }
    }
    return true;
}

void annotation::getSetOfClassesInDatasetAnnots(const std::vector<annotation::Annotation> annots, std::set<int> &classes_zoo)
{
    classes_zoo.clear();
    int n_obj;
    annotation::Annotation annot_tmp;
    annotation::SingleLineAnnotation sl_annot_tmp;
    for(int i = 0; i < annots.size(); ++i)
    {
        annot_tmp = annots[i];
        n_obj = annot_tmp.size();
        for(int j = 0; j < n_obj; ++j)
        {
            sl_annot_tmp = annot_tmp[j];
            classes_zoo.insert(sl_annot_tmp.get_class_id());
        }
    }
}
