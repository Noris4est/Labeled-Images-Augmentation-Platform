#ifndef TXT_FILES_PROCESSING_HPP
#define TXT_FILES_PROCESSING_HPP

#include <string>
#include <vector>

bool readTxtLineByLine(const std::string &path2file, std::vector<std::string> &fileContent);

//force_overwriting = true перезаписывает уже существующий файл (если он существует)
bool writeTxtLineByLine(const std::string &path2dstFile, const std::vector<std::string> &content, bool force_overwriting = true);

#endif //TXT_FILES_PROCESSING