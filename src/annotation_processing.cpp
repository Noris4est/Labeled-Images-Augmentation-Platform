
#include "annotation_processing.hpp"
bool parseAndCheckValidSingleLineAnnotation(
    const std::string &line, 
    int i_line, 
    const std::string &path2Txt, 
    annotation::SingleLineAnnotation &annot_s,
    bool &adaptationImnpemented)
{
    adaptationImnpemented = false; // флаг, указывающий, что была осуществлена небольшая подгонка разметки объекта
    std::string tmp_line = line;
    bool result = true;//предполагаем, что ошибок не возникнет
    string_processing::strip(tmp_line);//исключение(подрезание) пробелов в начеле и конце
    string_processing::deleteRepeatSeries(tmp_line, ' ');
    int class_id;
    double xc, yc, w, h;
    std::vector<std::string> substrings; // мб сделать static для экономии временнных ресурсов

    string_processing::split(tmp_line, substrings, ' '); // парсинг строки 
    if(substrings.size() != 5)
    {
        std::cout << "Warning: incorrect content in line №" << i_line << ". In file :" << path2Txt << std::endl 
            << "\t while splitting discovered too much information"  << std::endl;
        result = false;
    }
    class_id = std::stoi(substrings[0]);
    xc = std::stod(substrings[1]);
    yc = std::stod(substrings[2]);
    w = std::stod(substrings[3]);
    h = std::stod(substrings[4]);
    if(
        class_id < 0 || 
        xc < 0 || xc > 1 ||
        yc < 0 || yc > 1 ||
        w < 0 || w > 1 ||
        h < 0 || h > 1)
    {
        std::cout << "Warning: incorrect content in line №" << i_line << ". In file :" << path2Txt << std::endl 
            << " \t incorrect data (check class_id, xc, yc, w, h)"  << std::endl;
        result = false;
    }

    annot_s = annotation::SingleLineAnnotation(class_id, xc, yc, w, h);
    
    if(!annot_s.CheckValidAndAdaptation())
    {
        adaptationImnpemented = true;
    }
    return result;
}

/*
    Парсинг и проверка на валидность разметки одного объекта (одна строчка файла разметки).
    Принимается на вход строка, ее номер и путь к файлу(для сообщений об ошибках) и маркированный bbox.
    Рассчитывается lrbbox, возвращается флаг успешности операции (true/false) 
*/
bool parseAndCheckValidSingleLineAnnotation(
    const std::string &line, 
    int i_line, 
    const std::string &path2Txt, 
    LabledRelBbox &lrbbox,
    bool &adaptationImnpemented)
{
    annotation::SingleLineAnnotation annot_s;
    bool result = parseAndCheckValidSingleLineAnnotation(line, i_line, path2Txt, annot_s, adaptationImnpemented);
    lrbbox = annot_s.to_lrbbox();
    return result;
}
 // -- END parseAndCheckValidSingleLineMarkup