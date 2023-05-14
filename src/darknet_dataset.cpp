#include "darknet_dataset.hpp"
#include "txt_files_processing.hpp"
#include "string_processing.hpp"
#include "annotation.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <map>
#include <set>
#include <functional>
#include "ranges_processing.hpp"
#include <numeric>
#include <algorithm>
#include <random>
#include "rect_processing.hpp"
#include "cv_resize_tools.hpp"
#include "marked_frame.hpp"

using namespace darknet_dataset;

darknet_dataset::Dataset::Dataset()
{
    state = DatasetState::WAIT_SET_PATH;
}

bool Dataset::setPath2DatasetDir(
    const std::string &path2datasetDir,
    const std::set<std::string> &validFrameExtSet,
    const std::set<std::string> &validAnnotExtSet)
{
    if(!path_processing::isDirExist(path2datasetDir))
    {
        return false;
    }
    std::vector<std::string> framesDirContent, annotsDirContent;

    if(!path_processing::getDirFilesContent(path2datasetDir, validFrameExtSet, rawVecPath2frames, true))
    {
        return false;
    }

    if(!path_processing::getDirFilesContent(path2datasetDir, validAnnotExtSet, rawVecPath2annots, true))
    {
        return false;
    }
    
    state = DatasetState::WAIT_LAUNCH_CHECK_VALID;
    return true;
}

bool darknet_dataset::Dataset::setPath2txtFileListPath2datasetFrames(
    const std::string &path2framesTxtFileList, 
    const std::set<std::string> &validAnnotExtSet)
{    
    if(!path_processing::isRegularFileExist(path2framesTxtFileList))
    {
        return false;
    }
    std::vector<std::string> lblFileContent;
    if(!readTxtLineByLine(path2framesTxtFileList, lblFileContent))
    {
        return false;
    }
    rawVecPath2annots.clear();
    rawVecPath2annots.shrink_to_fit();
    rawVecPath2annots.reserve(lblFileContent.size());

    rawVecPath2frames.clear();
    rawVecPath2frames.shrink_to_fit();
    rawVecPath2frames.reserve(lblFileContent.size());

    std::string mod_path;
    DetailFilePath dfp, dfp_tmp;
    std::vector<std::string> tmp_annot_content;
    for(const std::string &path : lblFileContent)
    {
        mod_path = path; // modified path
        string_processing::strip(mod_path);
        dfp = DetailFilePath(mod_path);
        rawVecPath2frames.push_back(path);

        if(!path_processing::getDirFilesContent(dfp.dir, validAnnotExtSet, tmp_annot_content, true))
        {
            return false;
        }

        for(int i = 0; i < tmp_annot_content.size(); ++i)
        {
            dfp_tmp = DetailFilePath(tmp_annot_content[i]);
            if(dfp_tmp.clear_name == dfp.clear_name)
            {
                rawVecPath2annots.push_back(tmp_annot_content[i]);
            }
        }        
    }
    state = DatasetState::WAIT_LAUNCH_CHECK_VALID;
    return true;
}

bool darknet_dataset::Dataset::setPath2DatasetDirsToFramesAndAnnots(const std::string &path2framesDir, const std::string &path2annotsDir, const std::set<std::string> &validFrameExtSet, const std::set<std::string> &validAnnotExtSet)
{
    if(!path_processing::isDirExist(path2framesDir))
    {
        return false;
    }
    if(!path_processing::isDirExist(path2annotsDir))
    {
        return false;
    }
    std::vector<std::string> framesDirContent, annotsDirContent;

    if(!path_processing::getDirFilesContent(path2framesDir, validFrameExtSet, rawVecPath2frames, true))
    {
        return false;
    }

    if(!path_processing::getDirFilesContent(path2annotsDir, validAnnotExtSet, rawVecPath2annots, true))
    {
        return false;
    }
    
    state = DatasetState::WAIT_LAUNCH_CHECK_VALID;
    frames_and_annots_in_different_dirs = true;
    return true;
}

bool darknet_dataset::Dataset::setPath2txtFileListPath2datasetAnnots(const std::string &path2annotsTxtFileList, const std::set<std::string> &validFrameExtSet)
{
    if(!path_processing::isRegularFileExist(path2annotsTxtFileList))
    {
        return false;
    }
    std::vector<std::string> lblFileContent;
    if(!readTxtLineByLine(path2annotsTxtFileList, lblFileContent))
    {
        return false;
    }
    rawVecPath2annots.clear();
    rawVecPath2annots.shrink_to_fit();
    rawVecPath2annots.reserve(lblFileContent.size());

    rawVecPath2frames.clear();
    rawVecPath2frames.shrink_to_fit();
    rawVecPath2frames.reserve(lblFileContent.size());

    std::string mod_path;
    DetailFilePath dfp, dfp_tmp;
    std::vector<std::string> tmp_frames_content;
    for(const std::string &path : lblFileContent)
    {
        mod_path = path; // modified path
        string_processing::strip(mod_path);
        dfp = DetailFilePath(mod_path);
        rawVecPath2frames.push_back(path);

        if(!path_processing::getDirFilesContent(dfp.dir, validFrameExtSet, tmp_frames_content, true))
        {
            return false;
        }

        for(int i = 0; i < tmp_frames_content.size(); ++i)
        {
            dfp_tmp = DetailFilePath(tmp_frames_content[i]);
            if(dfp_tmp.clear_name == dfp.clear_name)
            {
                rawVecPath2annots.push_back(tmp_frames_content[i]);
            }
        }        
    }
    state = DatasetState::WAIT_LAUNCH_CHECK_VALID;
    return true;
}

bool darknet_dataset::Dataset::checkValid()
{
    state = DatasetState::NO_SUCCESS_VALID;

    // По этапам проверки рассматриваемые пути к фреймам и аннотациям будут сокращаться как в воронке

    std::function<bool(const std::string &)> isRegFile = [](const std::string &path2file)
    {
        return path_processing::isRegularFileExist(path2file); // возвращает статус успешности операции проверки на валидность аннотации
    };

    std::function<bool(const std::string &)> isValidAnnot = [](const std::string &path2annot)
    {
        annotation::Annotation annot;
        bool status = annot.readTxt(path2annot);
        return status; // возвращает статус успешности операции проверки на валидность аннотации
    };

    std::function<bool(const std::string &)> isValidFrame = [](const std::string &path2frame)
    {
        cv::Mat frame = cv::imread(path2frame);
        return !frame.empty();
    };


    std::vector<std::string> workVecPath2frames = rawVecPath2frames, workVecPath2annots = rawVecPath2annots;

    if(!path_processing::getUniqElements(rawVecPath2frames, workVecPath2frames, warnings.notUniqSrsFramesPaths)) // фильтрация на уникальные элементы
    {
        return false;
    }
    if(!path_processing::getUniqElements(rawVecPath2annots, workVecPath2annots, warnings.notUniqSrcAnnotsPaths))
    {
        return false;
    }
    if(!path_processing::filterPathsContentTrueCallback(workVecPath2frames, isRegFile, workVecPath2frames, errors.notExistSrcFramesPaths))
    {
        return false;
    }
    if(!path_processing::filterPathsContentTrueCallback(workVecPath2annots, isRegFile, workVecPath2annots, errors.notExistSrcAnnotsPaths))
    {
        return false;
    }
    
    if(!path_processing::filterPathsContentTrueCallback(workVecPath2frames, isValidFrame, workVecPath2frames, errors.notValidSrcFramesPaths))
    {
        return false;
    }
    if(!path_processing::filterPathsContentTrueCallback(workVecPath2annots, isValidAnnot, workVecPath2annots, errors.notValidSrsAnnotsPaths))
    {
        return false;
    }

    if(!path_processing::filterPathsContentSameNamesDifferentExtensions(workVecPath2frames, workVecPath2frames, errors.srcFramesWithMultiExts))
    {
        return false;
    }
    if(!path_processing::filterPathsContentSameNamesDifferentExtensions(workVecPath2annots, workVecPath2annots, errors.srcAnnotsWithMultiExts))
    {
        return false;
    }

    if(!path_processing::filterPathsContentMatchSamplesByPathWithoutExt(
        workVecPath2frames, workVecPath2annots, 
        workVecPath2frames, workVecPath2annots, 
        errors.srcFramesWithoutAnnotPair, errors.srcAnnotsWithoutFramePair,
        frames_and_annots_in_different_dirs))
    {
        return false;
    }

    if(workVecPath2frames.size() != workVecPath2annots.size())
    {
        return false;
    }

    cleaned_dataset.clear();
    cleaned_dataset.shrink_to_fit();
    cleaned_dataset.reserve(rawVecPath2frames.size());
    DatasetSample ds_tmp;
    for(int i = 0; i < workVecPath2annots.size(); ++i)
    {
        ds_tmp.annotPath = DetailFilePath(workVecPath2annots[i]);
        ds_tmp.framePath = DetailFilePath(workVecPath2frames[i]);
        cleaned_dataset.push_back(ds_tmp);
    }
    state = DatasetState::SUCCESS_VALID;
    return true;
}

bool darknet_dataset::Dataset::isValid()
{
    return state == DatasetState::SUCCESS_VALID;
}

int darknet_dataset::Dataset::get_total()
{
    return cleaned_dataset.size();
}

bool darknet_dataset::Dataset::getRawDataset(std::vector<std::string> &vecPath2frames, std::vector<std::string> &vecPath2annots)
{
    if(&vecPath2frames == &vecPath2annots)
    {
        throw std::runtime_error("Error in getValidDataset: &vecPath2frames == &vecPath2annots");
    }
    if(state == DatasetState::WAIT_SET_PATH)
    {
        return false;
    }

    vecPath2frames = rawVecPath2frames;
    vecPath2annots = rawVecPath2annots;
    return true;
}

bool darknet_dataset::Dataset::getValidDataset(std::vector<std::string> &vecPath2frames, std::vector<std::string> &vecPath2annots)
{
    if(&vecPath2frames == &vecPath2annots)
    {
        throw std::runtime_error("Error in getValidDataset: &vecPath2frames == &vecPath2annots");
    }
    if(state != DatasetState::SUCCESS_VALID)
    {
        return false;
    }
    vecPath2frames.clear();
    vecPath2frames.shrink_to_fit();
    vecPath2frames.reserve(cleaned_dataset.size());

    vecPath2annots.clear();
    vecPath2annots.shrink_to_fit();
    vecPath2annots.reserve(cleaned_dataset.size());

    for(int i = 0; i < cleaned_dataset.size(); ++i)
    {
        vecPath2annots.push_back(cleaned_dataset[i].annotPath.to_full_path());
        vecPath2frames.push_back(cleaned_dataset[i].framePath.to_full_path());
    }
    return true;
}

bool darknet_dataset::Dataset::getValidDatasetInIdsRange(int begin_id, int end_id, int step, std::vector<std::string> &vecPath2frames, std::vector<std::string> &vecPath2annots)
{
    if(&vecPath2frames == &vecPath2annots)
    {
        throw std::runtime_error("Error in getValidDataset: &vecPath2frames == &vecPath2annots");
    }
    if(state != DatasetState::SUCCESS_VALID)
    {
        return false;
    }
    assert(begin_id >= 0 && end_id < cleaned_dataset.size() && begin_id <= end_id && step > 0);
    std::vector<int> select_id_vec = range<int>(begin_id, end_id, step);
    vecPath2frames.clear();
    vecPath2frames.shrink_to_fit();
    vecPath2frames.reserve(cleaned_dataset.size());

    vecPath2annots.clear();
    vecPath2annots.shrink_to_fit();
    vecPath2annots.reserve(cleaned_dataset.size());
    int id_tmp;
    for(int i = 0; i < select_id_vec.size(); ++i)
    {
        id_tmp = select_id_vec[i];
        vecPath2annots.push_back(cleaned_dataset[id_tmp].annotPath.to_full_path());
        vecPath2frames.push_back(cleaned_dataset[id_tmp].framePath.to_full_path());   
    }
    return true;
}

void darknet_dataset::Dataset::get_random_subdataset(float partition, darknet_dataset::Dataset &child_dataset)
{
    if(state != DatasetState::SUCCESS_VALID)
    {
        throw std::runtime_error("Error: get_random_subdataset can be used only if dataset state is SUCCESS_VALID");
    }
    std::function<int64_t(double)> round_d2i = [](double x) 
    {
        return static_cast<int64_t>(x + 0.5 - static_cast<double>(x < 0));
    }; // round double to integer 64_t
    int src_dataset_size = cleaned_dataset.size();
    int dst_dataset_size = round_d2i(partition * src_dataset_size);
    if(dst_dataset_size > src_dataset_size)
    {
        dst_dataset_size = src_dataset_size;
    }
    std::vector<int> vector_of_indexes(src_dataset_size);
    std::iota(vector_of_indexes.begin(), vector_of_indexes.end(), 0);
    // get a time-based seed
    // size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    // shuffle (v.begin(), v.end(), std::default_random_engine(seed));
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(vector_of_indexes.begin(), vector_of_indexes.end(), g);

    child_dataset = Dataset();
    auto &ch_dataset_data = child_dataset.cleaned_dataset;
    ch_dataset_data.reserve(dst_dataset_size);
    for(int i = 0; i < dst_dataset_size; ++i)
    {
        ch_dataset_data.push_back(cleaned_dataset[vector_of_indexes[i]]);
    }
    child_dataset.is_child_dataset = true;
    child_dataset.state = DatasetState::SUCCESS_VALID;
}

void darknet_dataset::Dataset::get_labeled_object_collection(
    std::vector<darknet_dataset::dataset_object::DarknetLabeledObject> &labeled_objects_collection)
{
    if(state != DatasetState::SUCCESS_VALID)
    {
        throw std::runtime_error("Error: get_random_subdataset can be used only if dataset state is SUCCESS_VALID");
    }
    labeled_objects_collection.clear();
    labeled_objects_collection.shrink_to_fit();
    labeled_objects_collection.reserve(cleaned_dataset.size());
    darknet_dataset::dataset_object::DarknetLabeledObject dlo_tmp;
    std::string frame_path, annot_path;
    DatasetSample cl_elem;
    for(auto it = cleaned_dataset.cbegin(); it != cleaned_dataset.cend(); ++it)
    {
        cl_elem = *it;
        frame_path = cl_elem.framePath.to_full_path();
        annot_path = cl_elem.annotPath.to_full_path();
    }
}

DatasetErrorsElements darknet_dataset::Dataset::getErrors()
{
    return errors;
}

void darknet_dataset::Dataset::save_resized_dataset(
    const std::string &path2dst_dir, 
    cv::Size final_size, 
    bool make_dir_if_not_exist, 
    bool save_proportions, 
    cv::InterpolationFlags interp_type,
    bool dst_dir_must_be_empty)
{
    if(state != DatasetState::SUCCESS_VALID)
    {
        throw std::runtime_error("Error: get_random_subdataset can be used only if dataset state is SUCCESS_VALID");
    }

    if(!path_processing::isDirExist(path2dst_dir))
    {
        if(make_dir_if_not_exist)
        {
            path_processing::makePath(path2dst_dir);
        }
        else
        {
            throw std::runtime_error("Error: path2dst_dir not exist");
        }
    }
    else
    {
        if(dst_dir_must_be_empty)
        {
            if(!path_processing::isDirEmpty(path2dst_dir, true, true))
            {
                throw std::runtime_error("Error: path2dst_dir not empty!");
            }
        }
    }

    assert(final_size.width > 0 && final_size.height > 0);

    cv::Mat frame;
    annotation::Annotation annotation;
    std::string path2dst_frame;
    std::string path2dst_annotation;
    std::string cp_command;
    MarkedFrame mf_tmp;
    if(save_proportions)
    {
        for(const DatasetSample &d_sample : cleaned_dataset)
        {
            mf_tmp.set_frame_and_annot_path(
                d_sample.framePath.to_full_path(), 
                d_sample.annotPath.to_full_path());
            mf_tmp.resize_save_proportions(final_size);
            mf_tmp.save(path2dst_dir, d_sample.annotPath.clear_name);
        }
    }
    else
    {
        for(const DatasetSample &d_sample : cleaned_dataset)
        {
            frame = cv::imread(d_sample.framePath.to_full_path());
            cv::resize(frame, frame, final_size, 0, 0, interp_type);
            path2dst_frame = path2dst_dir + "/" + d_sample.framePath.to_basename();
            cv::imwrite(path2dst_frame, frame, {cv::IMWRITE_JPEG_QUALITY, 100});
            cp_command = "cp " + d_sample.annotPath.to_full_path() + " " + path2dst_dir;
            system(cp_command.c_str());
        }
    }
    
}

darknet_dataset::dataset_object::DarknetLabeledObject::DarknetLabeledObject()
{
    class_id = 0;
    bbox_2i = cv::Rect2i();
    annotation_path_ptr = nullptr;
}

darknet_dataset::dataset_object::DarknetLabeledObject::DarknetLabeledObject(
    int class_id, 
    cv::Rect2i bbox_2i, 
    std::string const *annotation_path_ptr, 
    std::string const *frame_path_ptr) : 
        class_id(class_id), 
        bbox_2i(bbox_2i), 
        annotation_path_ptr(annotation_path_ptr), 
        frame_path_ptr(frame_path_ptr)
{

}

double darknet_dataset::dataset_object::DarknetLabeledObject::getMeanBboxSize(MeanType mean_type) const
{
    switch (mean_type)
    {
    case MeanType::ARITHMETIC:
        {
            return static_cast<double>(bbox_2i.width + bbox_2i.height) / 2;
        }
        break;
    case MeanType::GEOMETRIC:
        {
            return std::sqrt(static_cast<double>(bbox_2i.width * bbox_2i.height));
        }
        break;
    case MeanType::HARMONIC:
        {
            return 2 / ( 1 / static_cast<double>(bbox_2i.width) + 1 / static_cast<double>(bbox_2i.height) );
        }
    default:
        throw std::runtime_error("Error: not supported mean_type");
    }
}
