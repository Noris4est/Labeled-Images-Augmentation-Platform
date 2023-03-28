#include "frame_warp_processing.hpp"
#include "distortion_mesh_generator_zoo.hpp"

void imageMeshWarpPerspective(const cv::Mat &src, cv::Mat &dst, const cv::Mat &dstWarpMesh)
{
    dst = cv::Mat(getMeshSize(dstWarpMesh), CV_8UC3);

    cv::Size meshGridSize = {dstWarpMesh.cols - 1, dstWarpMesh.rows - 1}; 
    cv::Mat primeMesh = meshGenerator::createPrimeMesh(src.size(), meshGridSize);
    std::vector<cv::Point2f> tmp_prime_poly2f, tmp_warp_poly2f; // Первичный и искаженный полигоны
    cv::Mat tmp_invTransformMat; // матрица преобразования между 
    std::vector<cv::Point2f> includedPoints2f; // Вектор включенных точек в искаженный полигон
    std::vector<cv::Point2f>  primeCorrespondPoints2f; // Вектор соответствующих точек, включенных в первичный полигон
    cv::Point2i warp_p, prime_p;
    cv::Point2f prime_tmp;

    cv::Vec3b vec_11, vec_12, vec_21, vec_22, interValue; // извлекаемые трехканальные пикселы для интерполяции и итоговое интерполированное значение
    int x1, x2, y1, y2; // Временные переменные для билинейной интерполяции
    float xf, yf; //xf - x_float , yf - y_float; координаты текущей точки в пространстве первичного полигона для интерполяции
    for(int i = 0; i < (primeMesh.rows - 1); ++i)
    {
        for(int j = 0; j < (primeMesh.cols - 1); ++j)
        {
            /*
            Извлучение полигонов (исходного и искаженного) в формате вектора 4х
            вершин по topleft индексу. Исходный cv::Mat содержит все вершины сеток.
            */
            getPoly4(primeMesh, tmp_prime_poly2f, {j,i});
            getPoly4(dstWarpMesh, tmp_warp_poly2f, {j,i});

            /*
            Формирование матрицы перспективного преобразования
            из пространства искаженного полигона в пространство первичного полигона
            */
            tmp_invTransformMat = cv::getPerspectiveTransform(tmp_warp_poly2f, tmp_prime_poly2f);

            /*
            Получение вектора всех точек, заключенных в искаженном полигоне
            */
            getAllPolygonPoints(tmp_warp_poly2f, includedPoints2f);

            /*
            Перспкетивное преобразование полученного вектора точек в неискаженное пространство
            */
            cv::perspectiveTransform(includedPoints2f, primeCorrespondPoints2f, tmp_invTransformMat);

            for(int i_p = 0; i_p < includedPoints2f.size(); ++i_p) //итерация по всем точкам полигона
            {
                warp_p = (cv::Point2i)includedPoints2f[i_p];

                /*
                Точки из пространства искаженного полигона имеют соответствя
                между точками первичного полигона, но координаты последних 
                являются дробными. Для снижения погрешностей и исключения
                систематического снижения разрешения применяется билинейная интерполяция.
                */
                prime_tmp = primeCorrespondPoints2f[i_p];
                x1 = (int)prime_tmp.x;
                x2 = x1 + 1;
                y1 = (int)prime_tmp.y;
                y2 = y1 + 1;    
                if( false &&
                    x1 > 0 && x2 < src.cols - 1 &&
                    y1 > 0 && y2 < src.rows - 1)
                {
                    //интерполяция между 4 ближайшими соседями
                    xf = prime_tmp.x;
                    yf = prime_tmp.y;
                    
                    vec_11 = src.at<cv::Vec3b>(y1, x1);
                    vec_21 = src.at<cv::Vec3b>(y1, x2);
                    vec_12 = src.at<cv::Vec3b>(y2, x1);
                    vec_22 = src.at<cv::Vec3b>(y2, x2);

                    interValue = 
                        vec_11*(x2 - xf)*(y2 - yf) +
                        vec_21*(xf - x1)*(y2 - yf) + 
                        vec_12*(x2 - xf)*(yf - y1) +
                        vec_22*(xf - x1)*(yf - y1); // ref : wikipedia
                    dst.at<cv::Vec3b>(warp_p.y, warp_p.x) = interValue;
                }
                else
                {
                    // Если пиксел на краю, то он не интерполируется, а напрямую записывается с округлением и адаптацией координат
                    prime_p = (cv::Point2i)prime_tmp;
                    if(checkAndAdaptingPointInFrame(warp_p, dst.size()) && checkAndAdaptingPointInFrame(prime_p, src.size()))
                    {
                        dst.at<cv::Vec3b>(warp_p.y, warp_p.x) = src.at<cv::Vec3b>(prime_p.y, prime_p.x);
                    }
                }
            }
        }
    }
}

void imageMeshWarpAffine(const cv::Mat &src, cv::Mat &dst, const cv::Mat &dstWarpMesh)
{
    dst = cv::Mat(getMeshSize(dstWarpMesh), CV_8UC3);

    cv::Size meshGridSize = {dstWarpMesh.cols - 1, dstWarpMesh.rows - 1}; 
    cv::Mat primeMesh = meshGenerator::createPrimeMesh(src.size(), meshGridSize);
    std::vector<cv::Point2f> tmp_prime_poly2f, tmp_warp_poly2f; // Первичный и искаженный полигоны
    cv::Mat tmp_invTransformMat, tmp_invTransformMat23; // матрица преобразования между 
    std::vector<cv::Point2f> includedPoints2f; // Вектор включенных точек в искаженный полигон
    std::vector<cv::Point2f>  primeCorrespondPoints2f; // Вектор соответствующих точек, включенных в первичный полигон
    cv::Point2i warp_p, prime_p;
    cv::Point2f prime_tmp;

    cv::Vec3b vec_11, vec_12, vec_21, vec_22, interValue; // извлекаемые трехканальные пикселы для интерполяции и итоговое интерполированное значение
    int x1, x2, y1, y2; // Временные переменные для билинейной интерполяции
    float xf, yf; //xf - x_float , yf - y_float; координаты текущей точки в пространстве первичного полигона для интерполяции
    std::vector<std::vector<cv::Point2f>> arrayOfPoly3nodesPrime, arrayOfPoly3nodesWarp;
    for(int i = 0; i < (primeMesh.rows - 1); ++i)
    {
        for(int j = 0; j < (primeMesh.cols - 1); ++j)
        {
            /*
            Извлучение полигонов (исходного и искаженного) в формате вектора 4х
            вершин по topleft индексу. Исходный cv::Mat содержит все вершины сеток.
            */
            getPoly4(primeMesh, tmp_prime_poly2f, {j,i});
            getPoly4(dstWarpMesh, tmp_warp_poly2f, {j,i});

            getPair3nodesPolyFrom4nodesPoly(tmp_prime_poly2f, arrayOfPoly3nodesPrime, true);
            getPair3nodesPolyFrom4nodesPoly(tmp_warp_poly2f, arrayOfPoly3nodesWarp, true);

            for(int poly_counter = 0; poly_counter < arrayOfPoly3nodesPrime.size(); ++poly_counter)
            {
                tmp_warp_poly2f = arrayOfPoly3nodesWarp[poly_counter];
                tmp_prime_poly2f = arrayOfPoly3nodesPrime[poly_counter];
            
                /*
                Формирование матрицы аффинного преобразования
                из пространства искаженного полигона в пространство первичного полигона
                */
                tmp_invTransformMat23 = cv::getAffineTransform(tmp_warp_poly2f, tmp_prime_poly2f);
                convertfromAffine2perspectiveMatrix(tmp_invTransformMat23, tmp_invTransformMat);

                /*
                Получение вектора всех точек, заключенных в искаженном полигоне
                */
                getAllPolygonPoints(tmp_warp_poly2f, includedPoints2f);

                /*
                Перспкетивное преобразование полученного вектора точек в неискаженное пространство
                */
                cv::perspectiveTransform(includedPoints2f, primeCorrespondPoints2f, tmp_invTransformMat);
                for(int i_p = 0; i_p < includedPoints2f.size(); ++i_p) //итерация по всем точкам полигона
                {
                    warp_p = (cv::Point2i)includedPoints2f[i_p];

                    /*
                    Точки из пространства искаженного полигона имеют соответствя
                    между точками первичного полигона, но координаты последних 
                    являются дробными. Для снижения погрешностей и исключения
                    систематического снижения разрешения применяется билинейная интерполяция.
                    */
                    prime_tmp = primeCorrespondPoints2f[i_p];
                    x1 = (int)prime_tmp.x;
                    x2 = x1 + 1;
                    y1 = (int)prime_tmp.y;
                    y2 = y1 + 1;    
                    if( false &&
                        x1 > 0 && x2 < src.cols - 1 &&
                        y1 > 0 && y2 < src.rows - 1)
                    {
                        //интерполяция между 4 ближайшими соседями
                        xf = prime_tmp.x;
                        yf = prime_tmp.y;
                        
                        vec_11 = src.at<cv::Vec3b>(y1, x1);
                        vec_21 = src.at<cv::Vec3b>(y1, x2);
                        vec_12 = src.at<cv::Vec3b>(y2, x1);
                        vec_22 = src.at<cv::Vec3b>(y2, x2);

                        interValue = 
                            vec_11*(x2 - xf)*(y2 - yf) +
                            vec_21*(xf - x1)*(y2 - yf) + 
                            vec_12*(x2 - xf)*(yf - y1) +
                            vec_22*(xf - x1)*(yf - y1); // ref : wikipedia
                        dst.at<cv::Vec3b>(warp_p.y, warp_p.x) = interValue;
                    }
                    else
                    {
                        // Если пиксел на краю, то он не интерполируется, а напрямую записывается с округлением и адаптацией координат
                        prime_p = (cv::Point2i)prime_tmp;
                        if(checkAndAdaptingPointInFrame(warp_p, dst.size()) && checkAndAdaptingPointInFrame(prime_p, src.size()))
                        {
                            dst.at<cv::Vec3b>(warp_p.y, warp_p.x) = src.at<cv::Vec3b>(prime_p.y, prime_p.x);
                        }
                    }
                }
            }
        }
    }
}
