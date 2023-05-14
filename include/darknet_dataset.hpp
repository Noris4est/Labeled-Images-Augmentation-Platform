#ifndef DARKNET_DATASET_HPP
#define DARKNET_DATASET_HPP

#include <string>
#include <set>
#include <memory>
#include "path_processing.hpp"
#include "common_types_project.hpp"
#include <opencv2/imgproc.hpp>

/*
Класс предоставляет интерфейс для управления
датасетом в формате darknet для тренировки и валидации
нейросей YOLO.

Класс не сохраняет аннотации и изображения в ОЗУ.
Форма хранения датасета - пути к изображениям и аннотациям.

Императивный подход заключается в step-by-step работе с объектом класса Dataset
1. Создание объекта через функцию create(...)
2. Передача path к датасету (несколько возможных интерфейсов, функция имеет переопределение)
3. Вызов метода checkValid, который запускает внутренний механизм валидации датасета
4. При повторном вызове checkValid без изменения путей к датасету возвращается ранее полученный результат
*/

namespace darknet_dataset
{
    class Dataset;
    namespace dataset_object
    {
        struct DarknetLabeledObject;
    }

    namespace iterator
    {
        class DatasetIterator
        {
        private:
            DatasetIterator();

        public:
            friend std::shared_ptr<DatasetIterator> create(std::shared_ptr<Dataset> Dataset);
            bool init();
            bool reset();
            bool get_next(std::string &path2frame, std::string &path2annot);
            int get_total(); // общее количество пар фрейм+аннотация в датасете
        };
    }

    namespace profiler
    {
        class DatasetProfiler
        {
            public:
                bool get_distribution_class_population(std::vector<int> &distribution_dst);
                bool get_distribution_from_size();
            private:
                std::shared_ptr<Dataset> dataset;

        };
    }


    namespace dataset_object
    {
        struct DarknetLabeledObject
        {
            DarknetLabeledObject();
            DarknetLabeledObject(
                int class_id, 
                cv::Rect2i bbox_2i, 
                std::string const * annotation_path_ptr, 
                std::string const * frame_path_ptr);
            
            double getMeanBboxSize(MeanType mean_type = ARITHMETIC) const;
            
            //fields
            int class_id;
            cv::Rect2i bbox_2i;
            std::string const * annotation_path_ptr; // Для работы с вектором таких объектов указатель на аннотацию из исходного датасата, память под них здесь не выделяется
            std::string const * frame_path_ptr;
        };
    }


    struct DatasetErrorsElements
    {
        std::vector<std::string> notExistSrcFramesPaths; // Это может быть при инициализации путей через txt файл путей
        std::vector<std::string> notExistSrcAnnotsPaths;

        std::vector<std::string> notValidSrcFramesPaths; // фреймы, которые не удалось открыть или они битые
        std::vector<std::string> notValidSrsAnnotsPaths; // аннотации, которые не удовлетворяют требованиям де-факто стандарта darknet YOLO разметке

        std::vector<std::string> srcFramesWithMultiExts; // фреймы, которые существуют с одним именем, но разными расширениями
        std::vector<std::string> srcAnnotsWithMultiExts; // аннотации, которые существуют с одним именем, но разными расширениями

        std::vector<std::string> srcFramesWithoutAnnotPair; // фреймы без пары - аннотации
        std::vector<std::string> srcAnnotsWithoutFramePair; // аннотации без пары - фрейма
    };

    struct DatasetWarningsElements
    {
        std::vector<std::string> notUniqSrsFramesPaths; // в случае присутствия нескольких путей к одному и тому же объекту датасета, то фильтрацией оставляется один
        std::vector<std::string> notUniqSrcAnnotsPaths;
    };

    struct DatasetSample
    {
        DatasetSample()
        {
            annotPath = DetailFilePath();
            framePath = DetailFilePath();
        }
        DetailFilePath annotPath;
        DetailFilePath framePath;
    };

    class Dataset
    {
    public:
        Dataset();

        // Путь к директории датасета, в которой сосуществуют изображения и аннотации
        bool setPath2DatasetDir(
            const std::string &path2datasetDir,
            const std::set<std::string> &validFrameExtSet = {"jpg", "jpeg", "JPG", "JPEG"},
            const std::set<std::string> &validAnnotExtSet = {"txt", "data", "annot"});

        // Путь к директориям с фреймами и аннотациями, фреймы и аннотации считываются из разных директорий
        bool setPath2DatasetDirsToFramesAndAnnots(
            const std::string &path2framesDir,
            const std::string &path2annotsDir,
            const std::set<std::string> &validFrameExtSet = {"jpg", "jpeg", "JPG", "JPEG"},
            const std::set<std::string> &validAnnotExtSet = {"txt", "data", "annot"});

        // Путь к txt файлу со списком путей к фреймам датасета, причем для каждого фрейма файл аннотации ищется в его директории
        bool setPath2txtFileListPath2datasetFrames( 
            const std::string &path2framesTxtFileList,
            const std::set<std::string> &validAnnotExtSet = {"txt", "data", "annot"}); // соответствующие файлы аннотации для каждого переданного фрейма будут искаться в той же директории что и фото

        // Путь к txt файлу со списком путей к аннотации датасета, причем для каждой аннотации изображение ищется в его директории
        bool setPath2txtFileListPath2datasetAnnots(
            const std::string &path2annotsTxtFileList,
            const std::set<std::string> &validFrameExtSet = {"jpg", "jpeg", "JPG", "JPEG"});

        bool checkValid(); // if return false, then dataset not valid - not use!
        bool isValid();    // проверка на валидность датасета

        int get_total(); // Общее число валидных пар frame+annotation

        bool getRawDataset(std::vector<std::string> &vecPath2frames, std::vector<std::string> &vecPath2annots);

        bool getValidDataset(std::vector<std::string> &vecPath2frames, std::vector<std::string> &vecPath2annots); // возвращает валидный датасет, если он был успешно провалидирован

        bool getValidDatasetInIdsRange(
            int begin_id, int end_id, int step, 
            std::vector<std::string> &vecPath2frames, 
            std::vector<std::string> &vecPath2annots);

        void get_random_subdataset(float partition, darknet_dataset::Dataset &child_dataset); // Получить случайный субдатасет, partition - объемная дол
        void get_labeled_object_collection(std::vector<darknet_dataset::dataset_object::DarknetLabeledObject> &labeled_objects_collection_dst);
        DatasetErrorsElements getErrors();

        void save_resized_dataset(
            const std::string &path2dst_dir, 
            cv::Size final_size, 
            bool make_dir_if_not_exist = true,
            bool save_proportions = true, 
            cv::InterpolationFlags interp_type = cv::InterpolationFlags::INTER_CUBIC,
            bool dst_dir_must_be_empty = true);

        // friend Dataset operator + (const Dataset &lhs, const Dataset &rhs);
        // friend std::shared_ptr<Dataset> operator + (
        //     const std::shared_ptr<Dataset> &lhs_p, const std::shared_ptr<Dataset> &rhs_p);

    private:
        enum DatasetState
        {
            WAIT_SET_PATH, // Ожидание установки путей к датасету
            WAIT_LAUNCH_CHECK_VALID,
            SUCCESS_VALID,   // Валидация датсета прошла успешно
            NO_SUCCESS_VALID // Неуспешная валидация датасета
        };

        DatasetState state;
        DatasetErrorsElements errors;
        DatasetWarningsElements warnings;
        bool is_child_dataset = false;
        bool frames_and_annots_in_different_dirs = false; // для режима определения датасета через две директории: одна с фреймами, другая с аннотациями

        std::vector<std::string> rawVecPath2frames; // исходные пути к фреймам
        std::vector<std::string> rawVecPath2annots; // исходные пути к аннотациям

        std::vector<DatasetSample> cleaned_dataset; // фильтрованный датасет
    };
}
#endif // DARKNET_DATASET_HPP
