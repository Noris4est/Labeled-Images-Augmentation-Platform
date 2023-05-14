#include "path_processing.hpp"
#include <fstream>
#include <sys/stat.h>
#include <algorithm>
#include <dirent.h>
#include <string>
#include <map>
#include <stdexcept>

DetailFilePath::DetailFilePath()
{
}

DetailFilePath::DetailFilePath(std::string dir, std::string clear_name, std::string extension)
{
    while (dir.back() == '/' || dir.back() == '\\')
    {
        dir.pop_back();
    }
    while (clear_name.front() == '/' || clear_name.front() == '/')
    {
        clear_name.erase(clear_name.begin());
    }
    while (clear_name.back() == '/' || clear_name.back() == '/')
    {
        clear_name.pop_back();
    }
    while (extension.front() == '.')
    {
        extension.erase(extension.begin());
    }
    this->dir = dir;
    this->clear_name = clear_name;
    this->extension = extension;
}

DetailFilePath::DetailFilePath(const std::string &fullFilePath)
{
    std::string tmp_basename, tmp_clean_name;
    if (!path_processing::getFileDir(fullFilePath, dir))
    {
        throw std::runtime_error("Error: getFileDir return false value!\n");
    }
    if (!path_processing::getBaseName(fullFilePath, tmp_basename))
    {
        throw std::runtime_error("Error: getBaseName return false value!\n");
    }
    if (!path_processing::getFileExt(tmp_basename, extension))
    {
        throw std::runtime_error("Error: getFileExt return false value!\n");
    }
    if (!path_processing::removeExtension(tmp_basename, clear_name))
    {
        throw std::runtime_error("Error: removeExtension return false value!\n");
    }
}

std::string DetailFilePath::to_full_path() const
{
    return dir + "/" + clear_name + "." + extension;
}

std::string DetailFilePath::to_basename() const
{
    return clear_name + "." + extension;
}

std::string DetailFilePath::to_path_without_ext() const
{
    return dir + "/" + clear_name;
}

std::ostream &operator<<(std::ostream &os, const DetailFilePath &dfp)
{
    os << "[" << dfp.dir << ';' << dfp.clear_name << ';' << dfp.extension << "];\n";
    return os;
}

using namespace path_processing;

bool path_processing::isFileExist(const std::string &path)
{
    std::ifstream dataFile(path);
    return dataFile.good();
}

bool path_processing::isRegularFileExist(const std::string &path)
{
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISREG(sb.st_mode));
}

bool path_processing::isDirExist(const std::string &path)
{
#if defined(_WIN32)
    struct _stat info;
    if (_stat(path.c_str(), &info) != 0)
    {
        return false;
    }
    return (info.st_mode & _S_IFDIR) != 0;
#else
    struct stat info;
    if (stat(path.c_str(), &info) != 0)
    {
        return false;
    }
    return (info.st_mode & S_IFDIR) != 0;
#endif
}

bool path_processing::makePath(const std::string &path)
{
#if defined(_WIN32)
    int ret = _mkdir(path.c_str());
#else
    mode_t mode = 0755;
    int ret = mkdir(path.c_str(), mode);
#endif
    if (ret == 0)
        return true;

    switch (errno)
    {
    case ENOENT:
        // parent didn't exist, try to create it
        {
            int pos = path.find_last_of('/');
            if (pos == std::string::npos)
#if defined(_WIN32)
                pos = path.find_last_of('\\');
            if (pos == std::string::npos)
#endif
                return false;
            if (!makePath(path.substr(0, pos)))
                return false;
        }
        // now, try to create again
#if defined(_WIN32)
        return 0 == _mkdir(path.c_str());
#else
        return 0 == mkdir(path.c_str(), mode);
#endif

    case EEXIST:
        // done!
        return isDirExist(path);

    default:
        return false;
    }
}

bool path_processing::getBaseName(const std::string &filepath, std::string &dstBaseName)
{
    if(&filepath != &dstBaseName)
    {
        dstBaseName.clear();
    } // проверка на то, что передана одна и та же ссылка

    bool slashExist = false, dotExist = false;
    int last_slash_pos = filepath.find_last_of("/\\");
    if (last_slash_pos != std::string::npos)
    {
        slashExist = true;
    }

    int last_dot_pos = filepath.rfind('.');
    if (last_dot_pos != std::string::npos)
    {
        dotExist = true;
    }

    if (!slashExist)
    {
        dstBaseName = filepath;
        return true;
    }

    if (!dotExist || (dotExist && last_dot_pos < last_slash_pos))
    {
        dstBaseName = "";
        return true;
    }

    if (dotExist && last_dot_pos > last_slash_pos)
    {
        dstBaseName = filepath.substr(last_slash_pos + 1);
        return true;
    }

    dstBaseName = "";
    return false;
}

bool path_processing::getFileExt(const std::string &src, std::string &dstExt)
{
    if(&src != &dstExt)
    {
        dstExt.clear();   
    }

    size_t i = src.rfind('.');
    if (i == std::string::npos)
    {
        return false;
    }
    dstExt = src.substr(i + 1);
    return true;
}

bool path_processing::getFileDir(const std::string &srcPath2file, std::string &dstPath2fileDir)
{
    // Возвращает путь к файлу без последнего "/"; Файлом считается объект с расширением
    if(&srcPath2file != &dstPath2fileDir)
    {
        dstPath2fileDir.clear();
    }

    bool slashExist = false, dotExist = false;
    int last_slash_pos = srcPath2file.find_last_of("/\\");
    if (last_slash_pos != std::string::npos)
    {
        slashExist = true;
    }

    int last_dot_pos = srcPath2file.rfind('.');
    if (last_dot_pos != std::string::npos)
    {
        dotExist = true;
    }

    if (!slashExist)
    {
        dstPath2fileDir = "";
        return true;
    }

    if (!dotExist || (dotExist && last_dot_pos < last_slash_pos))
    {
        dstPath2fileDir = srcPath2file;
        // передана директория, т.к. нет расширения
        if (dstPath2fileDir.back() == '/' || dstPath2fileDir.back() == '\\')
        {
            dstPath2fileDir.pop_back();
        }
        return true;
    }

    if (dotExist && last_dot_pos > last_slash_pos)
    {
        dstPath2fileDir = srcPath2file.substr(0, last_slash_pos);
        return true;
    }

    dstPath2fileDir = "";
    return false;
}

/*
    Функция извлекает путь к файлу без расширения.
    Пример: srcPath = "/home/user/data/file1.txt" -> result = "/home/user/data/file1"
*/
bool path_processing::removeExtension(const std::string &srcPath, std::string &result)
{
    if(&srcPath != &result)
    {
        result.clear();
    }
    int dotPos = srcPath.rfind('.');
    if (dotPos == std::string::npos)
    {
        result = "";
        return false;
    }
    result = srcPath.substr(0, dotPos);
    return true;
} // -- getFilePathWithoutExtension

enum EntDtype // enterance - вхождение
{
    ET_FILE,
    ET_DIR
};

bool path_processing::getDirFilesContent(
    const std::string &srcDir,
    const std::set<std::string> &validExts,
    std::vector<std::string> &resultNamesContent,
    bool returnFullPathToElements)
{
    resultNamesContent.clear();
    DIR *dir;
    struct dirent *ent;
    std::string fileExt;
    std::string fileName;
    if ((dir = opendir(srcDir.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            if (ent->d_type == DT_REG)
            {
                fileName = static_cast<std::string>(ent->d_name);
                if (!getFileExt(fileName, fileExt))
                {
                    return false;
                }
                if (validExts.find(fileExt) != validExts.end())
                {
                    resultNamesContent.push_back(fileName);
                }
            }
        }
        closedir(dir);
    }
    else
    {
        return false;
    }
    if(returnFullPathToElements)
    {
        std::string srcDirTmp = srcDir;
        if(srcDirTmp.back() == '/' || srcDirTmp.back() == '\\')
        {
            srcDirTmp.pop_back();
        }
        for(std::string &elem : resultNamesContent)
        {
            elem = srcDirTmp + '/' + elem;
        }
    }
    std::sort(resultNamesContent.begin(), resultNamesContent.end());
    return true;
}

bool path_processing::getDirFilesContentAllRegulars(
    const std::string &srcDir, 
    std::vector<std::string> &resultNamesContent, 
    bool returnFullPathToElements)
{
    resultNamesContent.clear();
    DIR *dir;
    struct dirent *ent;
    std::string fileName;
    if ((dir = opendir(srcDir.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            if (ent->d_type == DT_REG)
            {
                fileName = static_cast<std::string>(ent->d_name);
                resultNamesContent.push_back(fileName);
            }
        }
        closedir(dir);
    }
    else
    {
        return false;
    }
    if(returnFullPathToElements)
    {
        std::string srcDirTmp = srcDir;
        if(srcDirTmp.back() == '/' || srcDirTmp.back() == '\\')
        {
            srcDirTmp.pop_back();
        }
        for(std::string &elem : resultNamesContent)
        {
            elem = srcDirTmp + '/' + elem;
        }
    }
    std::sort(resultNamesContent.begin(), resultNamesContent.end());
    return true;
}

bool path_processing::getDirsContentInDir(
    const std::string &srcDir, 
    std::vector<std::string> &resultNamesContent, 
    bool returnFullPathToElements)
{
    resultNamesContent.clear();
    DIR *dir;
    struct dirent *ent;
    std::string dirName;
    if ((dir = opendir(srcDir.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            if (ent->d_type == DT_DIR)
            {
                dirName = static_cast<std::string>(ent->d_name);
                resultNamesContent.push_back(dirName);
            }
        }
        closedir(dir);
    }
    else
    {
        return false;
    }
    if(returnFullPathToElements)
    {
        std::string srcDirTmp = srcDir;
        if(srcDirTmp.back() == '/' || srcDirTmp.back() == '\\')
        {
            srcDirTmp.pop_back();
        }
        for(std::string &elem : resultNamesContent)
        {
            elem = srcDirTmp + '/' + elem;
        }
    }
    std::sort(resultNamesContent.begin(), resultNamesContent.end());
    return true;
}

bool path_processing::isDirEmpty(
    const std::string &dir, 
    bool consider_reg_files, 
    bool consider_dirs)
{
    bool general_empty = true;
    bool empty_reg_files = isNoRegFilesInDir(dir);
    bool empty_dirs_in_dir = isNoDirsInDir(dir);
    if(consider_reg_files)
    {
        general_empty &= empty_reg_files;
    }
    if(consider_dirs)
    {
        general_empty &= empty_dirs_in_dir;
    }
    return general_empty;
}

bool path_processing::isNoRegFilesInDir(const std::string &srcDir)
{
    bool reg_files_exist = false;
    DIR *dir;
    struct dirent *ent;
    std::string fileName;
    if ((dir = opendir(srcDir.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            if (ent->d_type == DT_REG)
            {
                reg_files_exist = true;
                break;
            }
        }
        closedir(dir);
    }
    else
    {
        throw std::runtime_error("Error: failed open dir: \"" + srcDir + "\"");
    }
    return !reg_files_exist;
}

bool path_processing::isNoDirsInDir(const std::string &srcDir)
{
    bool dirs_exist = false;
    DIR *dir;
    struct dirent *ent;
    std::string fileName;
    if ((dir = opendir(srcDir.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            if (ent->d_type == DT_DIR && static_cast<std::string>(ent->d_name) != "." && static_cast<std::string>(ent->d_name) != "..")
            {
                dirs_exist = true;
                break;
            }
        }
        closedir(dir);
    }
    else
    {
        throw std::runtime_error("Error: failed open dir: \"" + srcDir + "\"");
    }
    return !dirs_exist;
}

/*
Фильтрация входного контента по признаку одноименности объектов, но с разным расширениями;
Если, например, несколько изображений в директории будут иметь одинаковое имя, но разное расширение, то это не является допустимым;
Если возвращается false, то функция отработала некорректно, такие результаты dst_content использовать нельзя
*/
bool path_processing::filterPathsContentSameNamesDifferentExtensions(
    const std::vector<std::string> &src_content, 
    std::vector<std::string> &dst_valid_content, 
    std::vector<std::string> &dst_no_valid_content) // при use_cross_matching=true игнорируется директория расположения файлов (warning: может вызвать конфликт имен контента)
{
    // проверка каждого объекта на наличие одноименных, но с разными расширениями фреймов
    // для этого создается map, в который заносятся пути
    // ключи dir+clear_name, set - набор расширений

    if(&src_content == &dst_no_valid_content || &dst_valid_content == &dst_no_valid_content)
    {
        return false; 
    } 

    /*
    Для корректной отработки ситуации, когда &src_content == &dst_valid_content
    в шапке исполнения функции недопустимо очищать dst_valid_content вектор,
    т.к. это очистит src_content.

    Вводится прокси контейнер dst_valid_content_proxy, в который заносятся все изменения,
    в хвосте исполнения функции перед return осуществляется swap между dst_valid_content_proxy и dst_valid_content 
    за оптимальное время без прямого копирования контента и клонирования вектора.
    */

    std::vector<std::string> dst_valid_content_proxy;
    dst_valid_content_proxy.reserve(src_content.size());

    std::map<std::string, std::set<std::string>> content_exts;
    std::string path_without_ext_tmp;
    std::string ext_tmp;
    std::string path_tmp;

    dst_no_valid_content.clear();
    dst_no_valid_content.shrink_to_fit();
    dst_no_valid_content.reserve(src_content.size());

    if(!path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsSetExt(src_content, content_exts))
    {
        return false;
    }

    for (const auto &pair : content_exts) // обход всех пар ключ-значение
    {
        if (pair.second.size() == 1) // ok! (одно расширение - валидно)
        {
            dst_valid_content_proxy.push_back(pair.first + "." + *pair.second.begin());
        }
        else
        {
            for (const auto &ext : pair.second)
            {
                dst_no_valid_content.push_back(pair.first + "." + ext);
            }
        }
    }
    dst_no_valid_content.shrink_to_fit();
    dst_valid_content.swap(dst_valid_content_proxy);

    return true;
}

bool path_processing::getClearName(const std::string &srcPath, std::string &result)
{
    if(!getBaseName(srcPath, result))
    {
        return false;
    }
    if(!removeExtension(result, result))
    {
        return false;
    }
    return true;
}

bool path_processing::filterPathsContentTrueCallback(
    const std::vector<std::string> &src_content,
    const std::function<bool(const std::string &)> &callback,
    std::vector<std::string> &dst_valid_content,
    std::vector<std::string> &dst_no_valid_content)
{
    if(&src_content == &dst_no_valid_content || &dst_valid_content == &dst_no_valid_content)
    {
        return false; 
    }

    std::vector<std::string> dst_valid_content_proxy;
    dst_valid_content_proxy.reserve(src_content.size());

    std::string path_tmp;

    dst_no_valid_content.clear();
    dst_no_valid_content.shrink_to_fit();
    dst_no_valid_content.reserve(src_content.size());

    for (int i = 0; i < src_content.size(); ++i)
    {
        path_tmp = src_content[i];
        if (callback(path_tmp))
        {
            dst_valid_content_proxy.push_back(path_tmp);
        }
        else
        {
            dst_no_valid_content.push_back(path_tmp);
        }
    }

    dst_no_valid_content.shrink_to_fit();
    dst_valid_content.swap(dst_valid_content_proxy);
    return true;
}

bool path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsExt(
    const std::vector<std::string> &content,
    std::map<std::string, std::string> &dst_map)
{
    dst_map.clear();
    std::string path_tmp, path_without_ext_tmp, ext_tmp;
    for (int i = 0; i < content.size(); ++i)
    {
        path_tmp = content[i];
        if (!path_processing::removeExtension(path_tmp, path_without_ext_tmp))
        {
            return false;
        }
        if (!path_processing::getFileExt(path_tmp, ext_tmp))
        {
            return false;
        }
        dst_map[path_without_ext_tmp] = ext_tmp;
    }
    return true;
}

bool path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsSetExt(
    const std::vector<std::string> &content,
    std::map<std::string, std::set<std::string>> &dst_map)
{
    dst_map.clear();
    std::string path_tmp, path_without_ext_tmp, ext_tmp;

    for (int i = 0; i < content.size(); ++i)
    {
        path_tmp = content[i];
        if (!path_processing::removeExtension(path_tmp, path_without_ext_tmp))
        {
            return false;
        }
        if (!path_processing::getFileExt(path_tmp, ext_tmp))
        {
            return false;
        }
        // проверка существования текущего ключа в map'e.
        if (dst_map.find(path_without_ext_tmp) == dst_map.end())
        {
            // not found
            dst_map[path_without_ext_tmp] = {ext_tmp};
        }
        else
        {
            // found
            dst_map[path_without_ext_tmp].insert(ext_tmp);
        }
    }
    return true;
}

bool path_processing::filterPathsContentMatchSamplesByPathWithoutExt(
    const std::vector<std::string> &src_content1,
    const std::vector<std::string> &src_content2,
    std::vector<std::string> &dst_valid_content1,
    std::vector<std::string> &dst_valid_content2,
    std::vector<std::string> &dst_no_valid_content1,
    std::vector<std::string> &dst_no_valid_content2,
    bool content1_and_content2_in_two_different_dirs) // content1_and_content2_in_two_different_dirs значит, что контент src_content1 строго в одной директории, а src_content2 строго в другой
{
    if(&src_content1 == &src_content2)
    {
        throw std::runtime_error("Error in filterPathsContentMatchSamplesByPathWithoutExt: &src_content1 == &src_content2");
    }

    if(&dst_valid_content1 == &dst_valid_content2)
    {
        throw std::runtime_error("Error in filterPathsContentMatchSamplesByPathWithoutExt: &dst_valid_content1 == &dst_valid_content2");
    }

    if(&dst_no_valid_content1 == &dst_no_valid_content2)
    {
        throw std::runtime_error("Error in filterPathsContentMatchSamplesByPathWithoutExt: &dst_no_valid_content == &dst_no_valid_content2");
    }

    if(&dst_valid_content1 == &dst_no_valid_content1 ||
        &dst_valid_content1 == &dst_no_valid_content2 ||
        &dst_valid_content2 == &dst_no_valid_content1 ||
        &dst_valid_content2 == &dst_no_valid_content2)
    {
        throw std::runtime_error("Error in filterPathsContentMatchSamplesByPathWithoutExt: dst_valid_content1{i} == dst_no_valid_content{j}, i = 1..2, j = 1..2");
    }

    if(&src_content1 == &dst_no_valid_content1 || 
        &src_content1 == &dst_no_valid_content2 ||
        &src_content2 == &dst_no_valid_content1 ||
        &src_content2 == &dst_no_valid_content2)
    {
        throw std::runtime_error("Error in filterPathsContentMatchSamplesByPathWithoutExt: src_content{i} == dst_no_valid_content{j}, i = 1..2, j = 1..2");
    }

    std::vector<std::string> dst_valid_content1_proxy, dst_valid_content2_proxy;
    dst_valid_content1_proxy.reserve(src_content1.size());
    dst_valid_content1_proxy.reserve(src_content2.size());

    dst_no_valid_content1.clear();
    dst_no_valid_content1.shrink_to_fit();
    dst_no_valid_content1.reserve(src_content1.size());

    dst_no_valid_content2.clear();
    dst_no_valid_content2.shrink_to_fit();
    dst_no_valid_content2.reserve(src_content2.size());

    if(content1_and_content2_in_two_different_dirs)
    {
        std::set<std::string> dirs_set_tmp;
        if(!path_processing::getSetOfFilesDirsFromContent(src_content1, dirs_set_tmp))
        {
            return false;
        }
        if(dirs_set_tmp.size() != 1)
        {
            throw std::runtime_error("Content is distributed in different directories");
        }
        if(!path_processing::getSetOfFilesDirsFromContent(src_content2, dirs_set_tmp))
        {
            return false;
        }
        if(dirs_set_tmp.size() != 1)
        {
            throw std::runtime_error("Content is distributed in different directories");
        }       
    }

    std::string content1dir, content2dir;
    if(content1_and_content2_in_two_different_dirs)
    {
        if(!path_processing::getFileDir(src_content1[0], content1dir))
        {
            return false;
        }
        if(!path_processing::getFileDir(src_content2[0], content2dir))
        {
            return false;
        }
    }

    std::vector<DetailFilePath> src_cont1_dfp, src_cont2_dfp, inters_cont_dfp; // dfp  - detail file path

    src_cont1_dfp.reserve(src_content1.size());
    src_cont2_dfp.reserve(src_content2.size());
    inters_cont_dfp.reserve(std::min(src_content1.size(), src_content2.size()));

    for (auto const &path : src_content1)
    {
        src_cont1_dfp.push_back(DetailFilePath(path));
    }

    for (auto const &path : src_content2)
    {
        src_cont2_dfp.push_back(DetailFilePath(path));
    }
    std::function<bool(const DetailFilePath &, const DetailFilePath &)> comp;
    
    if(content1_and_content2_in_two_different_dirs)
    {
        comp = [](const DetailFilePath &dp1, const DetailFilePath &dp2)
            {
                return (dp1.clear_name) < (dp2.clear_name);
            };
    }  
    else
    {
        comp = [](const DetailFilePath &dp1, const DetailFilePath &dp2)
            {
                return (dp1.dir + dp1.clear_name) < (dp2.dir + dp2.clear_name);
            };
    }
    std::sort(src_cont1_dfp.begin(), src_cont1_dfp.end(), comp);
    std::sort(src_cont2_dfp.begin(), src_cont2_dfp.end(), comp);

    std::set_intersection(
        src_cont1_dfp.begin(), src_cont1_dfp.end(), src_cont2_dfp.begin(), src_cont2_dfp.end(), std::back_inserter(inters_cont_dfp), comp);

    std::map<std::string, std::string> cont1_ext, cont2_ext; // контейнеры для быстрого получения расширения, по ключу - путь файла без расширения

    if(content1_and_content2_in_two_different_dirs)
    {
        std::vector<std::string> src_cont1_basename, src_cont2_basename;
        src_cont1_basename.reserve(src_content1.size());
        src_cont2_basename.reserve(src_content2.size());
        std::string path_tmp, basename_tmp;
        for(int i = 0; i < src_content1.size(); ++i)
        {
            path_tmp = src_content1[i];
            if(!path_processing::getBaseName(path_tmp, basename_tmp))
            {
                return false;
            }
            src_cont1_basename.push_back(basename_tmp);
        }
        for(int i = 0; i < src_content2.size(); ++i)
        {
            path_tmp = src_content2[i];
            if(!path_processing::getBaseName(path_tmp, basename_tmp))
            {
                return false;
            }
            src_cont2_basename.push_back(basename_tmp);
        }
        if (!path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsExt(src_cont1_basename, cont1_ext))
        {
            return false;
        }

        if (!path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsExt(src_cont2_basename, cont2_ext))
        {
            return false;
        }
    }
    else
    {
        if (!path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsExt(src_content1, cont1_ext))
        {
            return false;
        }

        if (!path_processing::getMapFromContent_keyIsPathWithoutExt_valueIsExt(src_content2, cont2_ext))
        {
            return false;
        }
    }

    std::string basename_without_ext_tmp, path_without_ext_tmp, ext_tmp1, ext_tmp2;

    if(content1_and_content2_in_two_different_dirs)
    {
        for(auto const &dfp_tmp : inters_cont_dfp)
        {
            basename_without_ext_tmp = dfp_tmp.clear_name;
            ext_tmp1 = cont1_ext[basename_without_ext_tmp];
            ext_tmp2 = cont2_ext[basename_without_ext_tmp];
            dst_valid_content1_proxy.push_back(content1dir + '/' + basename_without_ext_tmp + '.' + ext_tmp1);
            dst_valid_content2_proxy.push_back(content2dir + '/' + basename_without_ext_tmp + '.' + ext_tmp2);   
        }
    }
    else
    {
        for (auto const &dfp_tmp : inters_cont_dfp)
        {
            path_without_ext_tmp = dfp_tmp.to_path_without_ext();
            ext_tmp1 = cont1_ext[path_without_ext_tmp];
            ext_tmp2 = cont2_ext[path_without_ext_tmp];
            dst_valid_content1_proxy.push_back(path_without_ext_tmp + "." + ext_tmp1);
            dst_valid_content2_proxy.push_back(path_without_ext_tmp + "." + ext_tmp2);
        }
    }


    std::vector<DetailFilePath> diff_cont1_and_inters, diff_cont2_and_inters;
    std::set_difference(src_cont1_dfp.begin(), src_cont1_dfp.end(), inters_cont_dfp.begin(), inters_cont_dfp.end(), std::back_inserter(diff_cont1_and_inters), comp);
    std::set_difference(src_cont2_dfp.begin(), src_cont2_dfp.end(), inters_cont_dfp.begin(), inters_cont_dfp.end(), std::back_inserter(diff_cont2_and_inters), comp);

    for(auto const &dfp_tmp: diff_cont1_and_inters)
    {
        dst_no_valid_content1.push_back(dfp_tmp.to_full_path());
    }

    for(auto const &dfp_tmp: diff_cont2_and_inters)
    {
        dst_no_valid_content2.push_back(dfp_tmp.to_full_path());
    }  

    dst_no_valid_content1.shrink_to_fit();
    dst_no_valid_content2.shrink_to_fit();

    dst_valid_content1.swap(dst_valid_content1_proxy);
    dst_valid_content2.swap(dst_valid_content2_proxy);

    return true;
}

bool path_processing::getUniqElements(
    const std::vector<std::string> &src_content, 
    std::vector<std::string> &dst_uniq_content, 
    std::vector<std::string> &dst_not_uniq_entrances)
{
    if(&src_content == &dst_not_uniq_entrances)
    {
        throw std::runtime_error("path_processing::getUniqElements &src_content == &dst_not_uniq_entrances");
    }
    if(&dst_uniq_content == &dst_not_uniq_entrances)
    {
        throw std::runtime_error("path_processing::getUniqElements &dst_uniq_content == &dst_not_uniq_entrances");
    }
    std::vector<std::string> dst_uniq_content_proxy;
    dst_uniq_content_proxy.reserve(src_content.size());
    
    dst_not_uniq_entrances.clear();
    dst_not_uniq_entrances.shrink_to_fit();
    dst_not_uniq_entrances.reserve(src_content.size());

    std::map <std::string, int> map_content;
    for(auto it = src_content.begin(); it != src_content.end(); ++it)
    {
        if(map_content.find(*it) != map_content.end())
        {
            ++map_content[*it];
        }
        else
        {
            map_content[*it] = 1;
        }
    }
    for(auto const pair : map_content)
    {
        if(pair.second != 1)
        {
            dst_not_uniq_entrances.push_back(pair.first);
        }
        dst_uniq_content_proxy.push_back(pair.first); // несмотря на кажущееся противоречие, все корректно
    }
    dst_not_uniq_entrances.shrink_to_fit();
    dst_uniq_content.swap(dst_uniq_content_proxy);
    return true;
}

bool path_processing::getSetOfFilesDirsFromContent(const std::vector<std::string> &src_content, std::set<std::string> &dst_dirs_set)
{
    std::string tmp_ext;
    // check all content is files (exist extension)
    for(auto it = src_content.begin(); it != src_content.end(); ++it)
    {
        if(!path_processing::getFileExt(*it, tmp_ext))
        {
            return false;
        }
    }
    dst_dirs_set.clear();
    std::string tmp_dir;
    for(auto it = src_content.begin(); it != src_content.end(); ++it)
    {
        if(!path_processing::getFileDir(*it, tmp_dir))
        {
            return false;
        }
        dst_dirs_set.insert(tmp_dir);
    }
    return true;
}
