#include "txt_files_processing.hpp"

#include <fstream>
#include "path_processing.hpp"

bool readTxtLineByLine(const std::string &path2file, std::vector<std::string> &fileContent)
{
    fileContent.clear();
    std::ifstream file(path2file);
    if (file.is_open()) 
    {
        std::string line;
        while (std::getline(file, line)) 
        {
            fileContent.push_back(line);
        }
        file.close();
    }
    else
    {
        return false;
    }    
    return true;
}

bool writeTxtLineByLine(const std::string &path2dstFile, const std::vector<std::string> &content, bool force_overwriting)
{
    if(path_processing::isRegularFileExist(path2dstFile) && !force_overwriting)
    {
        return false;
    }
    std::ofstream file(path2dstFile, std::ios::trunc);
    if(!file.is_open())
    {
        return false;
    }
    if(content.size() == 0)
    {
        return true;
    }
    for(int i = 0; i < content.size() - 1; ++i)
    {
        file << content[i] << "\n";
    }
    file << content.back();
    file.close();
    return true;
}
