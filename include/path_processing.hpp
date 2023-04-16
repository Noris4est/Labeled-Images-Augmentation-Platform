#ifndef PATH_PROCESSING_H
#define PATH_PROCESSING_H

#include <string>
#include <set>
#include <vector>
#include <memory>
#include <sstream>
#include <iostream>
#include <functional>
#include <map>

struct DetailFilePath
{
    DetailFilePath();
    DetailFilePath(std::string dir, std::string clear_name, std::string extension);
    DetailFilePath(const std::string &fullFilePath);

    std::string to_full_path() const; // формирование полного пути
    std::string to_path_without_ext() const; // формирование пути без расширения
    std::string dir = "";        // file dir
    std::string clear_name = ""; // name without extension
    std::string extension = "";
};


std::ostream &operator<<(std::ostream &os, const DetailFilePath &dfp);

namespace path_processing
{
    bool isFileExist(const std::string &path); // есть путь указать к директории, то тоже возвращает true

    bool isRegularFileExist(const std::string &path);

    bool isDirExist(const std::string &path);

    bool makePath(const std::string &path);

    bool getBaseName(const std::string &filepath, std::string &dstBaseName);

    bool getFileExt(const std::string &src, std::string &dstExt);

    bool getFileDir(const std::string &src_Path2file, std::string &dst_path2fileDir); // extract path to directory from path to file

    bool removeExtension(const std::string &srcPath, std::string &result);

    bool getDirFilesContent(
        const std::string &srcDir,
        const std::set<std::string> &validExts,
        std::vector<std::string> &resultNamesContent,
        bool returnFullPathToElements = false);

    bool getDirFilesContentAllRegulars(
        const std::string &srcDir,
        std::vector<std::string> &resultNamesContent,
        bool returnFullPathToElements = false); // возвращает все регулярные файлы

    bool getDirsContentInDir(
        const std::string &srcDir,
        std::vector<std::string> &resultNamesContent,
        bool returnFullPathToElements = false); // возвращает все директории в указанной директории 
    
    bool isDirEmpty(
        const std::string &dir, 
        bool consider_reg_files = true, 
        bool consider_dirs = false); // consider_dirs - проверка с учетом директорий

    bool isNoRegFilesInDir(const std::string &dir); // Бросает исключение, если директория не найдена
    bool isNoDirsInDir(const std::string &dir);

    bool filterPathsContentSameNamesDifferentExtensions(
        const std::vector<std::string> &src_content, 
        std::vector<std::string> &dst_valid_content, 
        std::vector<std::string> &dst_no_valid_content);

    bool getClearName(const std::string &srcPath, std::string &result);
    /*
    Фильтрация исходного контента путей по признаку возвращаемого значения передаваемой callback функции, в dst_valid_content попадают образцы, для которых возвращаемое значение callback = true
    */
    bool filterPathsContentTrueCallback(
        const std::vector<std::string> &src_content,
        const std::function<bool(const std::string &)> &callback,
        std::vector<std::string> &dst_valid_content,
        std::vector<std::string> &dst_no_valid_content);
    
    /*
    Формирование хэш-таблицы из контента, представленного набором путей к некоторым файлам.
    Ключи map - пути файлов без расширения.
    Значения map - расширение этих файлов.
    Внимание! Если в исходной коллекции присутствуют объекты, пути к которым отличаются только расширениями,
    то в value map будет записано последнее расширение
    */

    bool getMapFromContent_keyIsPathWithoutExt_valueIsExt(
        const std::vector<std::string> &content,
        std::map<std::string, std::string> &dst_map);

    /*
    Формирование хэш-таблицы из контента, представленного набором путей к некоторым файлам.
    Ключи map - пути файлов без расширения.
    Значения map - набор (std::set) расширений этих файлов.
    */

    bool getMapFromContent_keyIsPathWithoutExt_valueIsSetExt(
        const std::vector<std::string> &content,
        std::map<std::string, std::set<std::string>> &dst_map);

    /*
    Фильтрация исходного контента, представленного в двух векторах путей к файлам. 
    Осуществляется матчинг (сопоставление) путей между коллекциями, при этом пути к файлам
    рассматриваются без расширений. 
    Например, path1 = "/home/user/dataset1/0001.png" и path2 = "/home/user/dataset1/0001.txt" успешно сопоставятся (успешный матчинг).
    Образцы из первой коллекции, для которых успешно найдена пара из второй коллекции, будут записаны в dst_valid_content1 (аналогично для второй коллекции)
    Образцы из первой коллекции, для которых не будет найдена пара из второй коллекции, будут записаны в dst_no_valid_content1 (аналогично для второй коллекции) 
    Если возвращается значение false, то во время работы произошла ошибка, выходные коллекции невалидны.
    */

    bool filterPathsContentMatchSamplesByPathWithoutExt(
        const std::vector<std::string> &src_content1, 
        const std::vector<std::string> &src_content2, 
        std::vector<std::string> &dst_valid_content1,
        std::vector<std::string> &dst_valid_content2,
        std::vector<std::string> &dst_no_valid_content1,
        std::vector<std::string> &dst_no_valid_content2,
        bool content1_and_content2_in_two_different_dirs);
    
    bool getUniqElements(
        const std::vector<std::string> &src_content,
        std::vector<std::string> &dst_uniq_content,
        std::vector<std::string> &dst_not_uniq_entrances);
    bool getSetOfFilesDirsFromContent(
        const std::vector<std::string> &src_content,
        std::set<std::string> &dst_dirs_set);
}
#endif // PATH_PROCESSING_H
