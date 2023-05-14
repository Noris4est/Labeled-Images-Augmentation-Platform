#include "warp_mesh_applicator.hpp"
#include "distortion_mesh_generator_zoo.hpp"
#include "polygon_processing.hpp"
#include "affine_processing.hpp"
#include "utils.hpp"
#include "rect_processing.hpp"

float calculate_relative_delta_EF(const std::vector<cv::Point> &poly, cv::Point point0, bool checkvalidpoly = true)
{
    if(checkvalidpoly)
    {
        assert(poly.size() == 4); // Допустимы исключительно полигоны с 4 вершинами!
        /*
            Полигон должен быть выпуклим и не вырожденным в треугольник 
        */
       assert(isConvexPolygon(poly, true, false));
    }
    
    cv::Point A = poly[0];
    cv::Point B = poly[1];
    cv::Point C = poly[2];
    cv::Point D = poly[3];
    int x0 = point0.x;
    int y0 = point0.y;
    cv::Vec2i AB, DC;
    AB = B - A;
    DC = C - D;
    int DCx = DC[0];
    int DCy = DC[1];
    int ABx = AB[0];
    int ABy = AB[1];

    long long A_eq = DCy*ABx - DCx*ABy;
    long long B_eq = ABy * (x0 - D.x) + ABx * (D.y - y0) + DCx * (y0 - A.y) - DCy * (x0 - A.x);
    long long C_eq = D.x * (y0 - A.y) + D.y * (A.x - x0) - A.x * y0 + A.y * x0;
    float delta;

    if(A_eq == 0)
    {
        delta = -(float) C_eq / B_eq;
    }
    else
    {
        long long D_eq = B_eq*B_eq - 4*A_eq*C_eq;
        
        float x1 = (-B_eq + std::sqrt(D_eq)) / 2.0 / A_eq;
        float x2 = (-B_eq - std::sqrt(D_eq)) / 2.0 / A_eq;
        float tolerance = 0.1;
        if(x1 >= -tolerance && x1 <= 1 + tolerance)
        {
            delta = x1;
        }
        else
        {
            delta = x2;
        }
    }
    return delta;
}

float calculate_relative_delta_GH_proxy(const std::vector<cv::Point> &poly, cv::Point point0, bool checkvalidpoly = true)
{
    if(checkvalidpoly)
    {
        assert(poly.size() == 4); // Допустимы исключительно полигоны с 4 вершинами!
        /*
            Полигон должен быть выпуклим и не вырожденным в треугольник 
        */
       assert(isConvexPolygon(poly, true, false));
    }
    
    cv::Point A = poly[0];
    cv::Point B = poly[1];
    cv::Point C = poly[2];
    cv::Point D = poly[3];
    int x0 = point0.x;
    int y0 = point0.y;
    cv::Vec2i AB, DC;
    AB = B - A;
    DC = C - D;
    int DCx = DC[0];
    int DCy = DC[1];
    int ABx = AB[0];
    int ABy = AB[1];

    long long A_eq = DCy*ABx - DCx*ABy;
    long long B_eq = ABy * (x0 - D.x) + ABx * (D.y - y0) + DCx * (y0 - A.y) - DCy * (x0 - A.x);
    long long C_eq = D.x * (y0 - A.y) + D.y * (A.x - x0) - A.x * y0 + A.y * x0;
    float delta;

    if(A_eq == 0)
    {
        delta = -(float) C_eq / B_eq;
    }
    else
    {
        long long D_eq = B_eq*B_eq - 4*A_eq*C_eq;
        
        float x1 = (-B_eq + std::sqrt(D_eq)) / 2.0 / A_eq;
        float x2 = (-B_eq - std::sqrt(D_eq)) / 2.0 / A_eq;
        float tolerance = 0.1;
        if(x1 >= -tolerance && x1 <= 1 + tolerance)
        {
            delta = x1;
        }
        else
        {
            delta = x2;
        }
    }
    return delta;
}

float calculate_relative_delta_GH(const std::vector<cv::Point> &poly, cv::Point point0, bool checkvalidpoly = true)
{
    auto poly_rot = poly;
    std::rotate(poly_rot.begin(), poly_rot.begin() + 1, poly_rot.end()); // циклический сдвиг влево на 1 позицию
    return calculate_relative_delta_EF(poly_rot, point0, checkvalidpoly);
}

std::pair<float, float> calculate_relative_delta12(const std::vector<cv::Point> &poly, cv::Point point0, bool checkvalidpoly = true)
{
    float d1 = calculate_relative_delta_EF(poly, point0, checkvalidpoly);
    float d2 = calculate_relative_delta_GH(poly, point0, checkvalidpoly);
    return {d1, d2};
}

cv::Point transformCoordinatesPoint4nodePoly(const std::vector<cv::Point> polySrc, const std::vector<cv::Point> polyDst, cv::Point srcPoint)
{
    auto d1d2 = calculate_relative_delta12(polySrc, srcPoint, false);
    float delta1 = d1d2.first;
    float delta2 = d1d2.second;
    cv::Point A = polyDst[0];
    cv::Point B = polyDst[1];
    cv::Point C = polyDst[2];
    cv::Point D = polyDst[3];
    cv::Vec2i AB, DC, BC, AD;
    AB = B - A;
    DC = C - D;
    BC = C - B;
    AD = D - A;
    int DCx = DC[0];
    int DCy = DC[1];
    int ABx = AB[0];
    int ABy = AB[1];
    int BCx = BC[0];
    int BCy = BC[1];
    int ADx = AD[0];
    int ADy = AD[1];

    cv::Point E = {A.x + int(delta1*ABx + 0.5), A.y + int(delta1*ABy + 0.5)};
    cv::Point F = {D.x + int(delta1*DCx + 0.5), D.y + int(delta1*DCy + 0.5)};

    cv::Point H = {B.x + int(delta2*BCx + 0.5), B.y + int(delta2*BCy+ 0.5)};
    cv::Point G = {A.x + int(delta2*ADx + 0.5), A.y + int(delta2*ADy+ 0.5)};
    cv::Point Pintersect;
    int denominator = ((E.x - F.x) * (H.y - G.y) - (E.y - F.y) * (H.x - G.x));
    Pintersect.x = (float)((E.x * F.y - E.y * F.x) * (H.x - G.x) - (E.x - F.x) * (H.x * G.y - H.y * G.x)) / denominator;
    Pintersect.y = (float)((E.x * F.y - E.y * F.x) * (H.y - G.y) - (E.y - F.y) * (H.x * G.y - H.y * G.x)) / denominator;
    return Pintersect;
}

MeshWarpApplicator::MeshWarpApplicator()
{
    
}

MeshWarpApplicator::MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, bool instantPreprocessing)
{
    this->srcMesh = srcMesh.clone();
    this->dstMesh = dstMesh.clone();
    meshGridSize = {(srcMesh.cols - 1) , (srcMesh.rows - 1)};
    framesize = srcMesh.at<cv::Point2i>(dstMesh.rows - 1, dstMesh.cols - 1);
    framesize.width += 1;
    framesize.height += 1;
    if(instantPreprocessing)
    {
        preprocessing();
    }
}

MeshWarpApplicator::MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, cv::Size framesize, cv::Size meshGridSize, bool instantPreprocessing)
{
    this->srcMesh = srcMesh.clone();
    this->dstMesh = dstMesh.clone();
    this->framesize = framesize;
    this->meshGridSize = meshGridSize;
    //meshGridSize = {(srcMesh.cols - 1) , (srcMesh.rows - 1)};
    if(instantPreprocessing)
    {
        perprocessing3nodes();
    }
}

void MeshWarpApplicator::setSrcDstMesh(const cv::Mat &srcMesh, const cv::Mat &dstMesh, bool instantPreprocessing)
{
    this->srcMesh = srcMesh.clone();
    this->dstMesh = dstMesh.clone();
    this->framesize = framesize;
    this->meshGridSize = meshGridSize;
    if(instantPreprocessing)
    {
        perprocessing3nodes();
    }
}

cv::Size MeshWarpApplicator::getMeshGridSize()
{
    return srcMesh.size();
}

const cv::Mat &MeshWarpApplicator::getWarpMesh()
{
    return dstMesh;
}

void MeshWarpApplicator::preprocessing()
{
    int totalPolygonsCount = meshGridSize.width * meshGridSize.height;
    
    //get last element dstMesh - size
    cv::Point2i dstMesh_br = dstMesh.at<cv::Point2i>(dstMesh.rows - 1, dstMesh.cols - 1); //br = bottom right
    cv::Size map_poly_size = {dstMesh_br.x + 1, dstMesh_br.y + 1};
    dst_frame_size = map_poly_size;
    mapPolygonIds = cv::Mat(map_poly_size, CV_16UC1);
    mapPolygonIds = cv::Scalar(0);
    cv::Point tl, tr, br, bl;
    std::vector<cv::Point> polygon_tmp;
    int id_counter = 1;

    for(int i = 0; i < meshGridSize.height; ++i)
    {
        for(int j = 0; j < meshGridSize.width; ++j)
        {
            extractPolygon4(dstMesh, polygon_tmp, {j,i});
            // Проверить полигон на валидность
            assert(isConvexPolygon(polygon_tmp, true, false));
            cv::fillConvexPoly(mapPolygonIds, polygon_tmp, cv::Scalar(id_counter++));
        }
    }

    // 0 - здесь и далее расценивается как ошибка
    // Здесь продумать, как можно работать с незакрашенными пикселами. Может быть, интерполяция ближайшим соседом.
    // int mapPolyArea = framesize.area();
    // std::cout << "Count zero pixels = " << mapPolyArea - cv::countNonZero(mapPolygonIds) << std::endl;
    int zeropixcounter = 0;
    for(int i = 0; i < mapPolygonIds.rows; ++i)
    {
        for(int j = 0; j < mapPolygonIds.cols; ++j)
        {
            int pixval = mapPolygonIds.at<ushort>(i, j);
            if(pixval == 0)
                ++zeropixcounter;
        }
    }
    //std::cout << "Count zero pixels = " << zeropixcounter << std::endl;


    uint16_t current_poly_id;
    int i_poly, j_poly;
    std::vector<cv::Point> polygon_dst_tmp, polygon_src_tmp;
    cv::Point pSrc;
    map_x_inverse = cv::Mat(dst_frame_size, CV_32FC1);
    map_y_inverse = cv::Mat(dst_frame_size, CV_32FC1);
    map_x_direct = cv::Mat(dst_frame_size, CV_32FC1);
    map_y_direct = cv::Mat(dst_frame_size, CV_32FC1);

    // fill map_x_inverse, map_y_inverse
    // scan dst_frame space
    for(int i = 0; i < framesize.height; ++i)
    {
        for(int j = 0; j < framesize.width; ++j)
        {
            // if(i == 100 && j == 375)
            // {
            //     std::cout << "Here!" << std::endl;
            // }
            current_poly_id = mapPolygonIds.at<ushort>(i,j);
            
            i_poly = int((current_poly_id-1) / meshGridSize.width);
            j_poly = (current_poly_id-1) % meshGridSize.width;
            extractPolygon4(dstMesh, polygon_dst_tmp, {j_poly, i_poly});
            extractPolygon4(srcMesh, polygon_src_tmp, {j_poly, i_poly});
            pSrc = transformCoordinatesPoint4nodePoly(polygon_dst_tmp, polygon_src_tmp, {j,i});
            map_x_inverse.at<float>(i, j) = pSrc.x;
            map_y_inverse.at<float>(i, j) = pSrc.y;
        }
    }

    //fill map_x_direct, map_y_direct
    //scan src_frame space
    int cell_height = framesize.height / meshGridSize.height;
    int cell_width = framesize.width / meshGridSize.width;
    cv::Point pDst;
    for(int i = 0; i < framesize.height; ++i)
    {
        for(int j = 0; j < framesize.width; ++j)
        {
            i_poly = i / cell_height;
            j_poly = j / cell_width;
            extractPolygon4(dstMesh, polygon_dst_tmp, {j_poly, i_poly});
            extractPolygon4(srcMesh, polygon_src_tmp, {j_poly, i_poly});
            pDst = transformCoordinatesPoint4nodePoly(polygon_src_tmp, polygon_dst_tmp, {j,i});
            map_x_direct.at<float>(i, j) = pDst.x;
            map_y_direct.at<float>(i, j) = pDst.y;
        }
    }
    is_preprocessed = true;
}

void MeshWarpApplicator::perprocessing3nodes()
{
    mapPolygonIds = cv::Mat(framesize, CV_16UC1);
    mapPolygonIds = cv::Scalar(0);
    int id_counter = 1;
    std::vector<cv::Point> polygon_tmp;

    for(int i = 0; i < meshGridSize.height; ++i)
    {
        for(int j = 0; j < meshGridSize.width; ++j)
        {
            extractPolygon3(dstMesh, polygon_tmp, {j,i});
            // Проверить полигон на валидность
            assert(isConvexPolygon(polygon_tmp, true, false));
            cv::fillConvexPoly(mapPolygonIds, polygon_tmp, cv::Scalar(id_counter++));
        }
    }
    int zeropixcounter = 0;
    for(int i = 0; i < mapPolygonIds.rows; ++i)
    {
        for(int j = 0; j < mapPolygonIds.cols; ++j)
        {
            int pixval = mapPolygonIds.at<ushort>(i, j);
            if(pixval == 0)
                ++zeropixcounter;
        }
    }
    std::cout << "Count zero pixels = " << zeropixcounter << std::endl;

    uint16_t current_poly_id;
    int i_poly, j_poly;
    std::vector<cv::Point> polygon_dst_tmp, polygon_src_tmp;
    cv::Point pSrc;
    map_x_inverse = cv::Mat(framesize, CV_32FC1);
    map_y_inverse = cv::Mat(framesize, CV_32FC1);

    std::vector<cv::Mat> affineMatrixVec;

    
    for(int i = 0; i < meshGridSize.height; ++i)
    {
        for(int j = 0; j < meshGridSize.width; ++j)
        {
            extractPolygon3(dstMesh, polygon_dst_tmp, {j, i});
            extractPolygon3(srcMesh, polygon_src_tmp, {j, i});
            
            affineMatrixVec.push_back(cv::getAffineTransform(
                convertIntPolygon2Float(polygon_dst_tmp), 
                convertIntPolygon2Float(polygon_src_tmp)));
        }
    }

    for(int i = 0; i < framesize.height; ++i)
    {
        for(int j = 0; j < framesize.width; ++j)
        {
            // if(i == 314 && j == 99)
            // {
            //     std::cout << "Here!" << std::endl;
            // }
            current_poly_id = mapPolygonIds.at<ushort>(i,j);
            i_poly = int((current_poly_id-1) / meshGridSize.width);
            j_poly = (current_poly_id-1) % meshGridSize.width;
            pSrc = warpAffine2Point<cv::Point2i>(cv::Point(j,i), affineMatrixVec[current_poly_id - 1]);
            // cv::perspectiveTransform({j,i}, pSrc, affineMatrixVec[current_poly_id - 1]);
            //pSrc = transformCoordinatesPoint4nodePoly(polygon_dst_tmp, polygon_src_tmp, {j,i});
            map_x_inverse.at<float>(i, j) = pSrc.x;
            map_y_inverse.at<float>(i, j) = pSrc.y;
        }
    }
}

void MeshWarpApplicator::apply(const cv::Mat &src, cv::Mat &dst) const
{
    assert(is_preprocessed);
    assert(!src.empty());
    cv::Mat src_resized;
    if(src.size() != dst_frame_size)
    {
        cv::resize(src, src_resized, dst_frame_size, 0, 0, cv::InterpolationFlags::INTER_CUBIC);
    }
    else
    {
        src_resized = src;
    }
    cv::remap(src_resized, dst, map_x_inverse, map_y_inverse, cv::InterpolationFlags::INTER_CUBIC);
}

void MeshWarpApplicator::apply(const MarkedFrame &src, MarkedFrame &dst) const
{
    assert(is_preprocessed);
    assert(!src.isFrameEmpty());
    
    MarkedFrame src_adapted;
    src_adapted.set_frame(src.getFrameRef());
    src_adapted.set_annotation(src.getAnnotRef());

    if(src.size() != dst_frame_size)
    {
        src_adapted.resize(dst_frame_size);
    }

    cv::remap(src_adapted.getFrameRef(), dst.getFrameRef(), map_x_inverse, map_y_inverse, cv::InterpolationFlags::INTER_CUBIC);

    const annotation::Annotation &annot_src = src_adapted.getAnnotRef();
    annotation::Annotation &annot_dst = dst.getAnnotRef();
    const cv::Mat &frame_src = src_adapted.getFrameRef();
    cv::Size src_frame_size = frame_src.size();
    cv::Rect2i bbox2i, bbox2i_dst;
    annotation::SingleLineAnnotation sl_annot;
    std::vector<cv::Point2i> bbox_pts, bbox_pts_dst;
    cv::Point p_dst;
    cv::Rect2d bbox2d_dst;
    annot_dst.clear();
    for(int i = 0; i < annot_src.size(); ++i)
    {
        sl_annot = annot_src[i];
        bbox2i = sl_annot.to_rect_2i(src_frame_size);
        getRectPoints(bbox2i, bbox_pts, true);
        bbox_pts_dst.clear();
        for(cv::Point p_tmp : bbox_pts)
        {
            p_dst.x = map_x_direct.at<float>(p_tmp.y, p_tmp.x);
            p_dst.y = map_y_direct.at<float>(p_tmp.y, p_tmp.x);
            bbox_pts_dst.push_back(p_dst);
        }
        bbox2i_dst = cv::boundingRect(bbox_pts_dst);
        bbox2d_dst = getRelRectFromAbs(bbox2i_dst, frame_src.size());
        sl_annot.set_rect(bbox2d_dst);
        annot_dst.push_back(sl_annot);
    }
}

bool MeshWarpApplicator::checkValidMesh(const cv::Mat mesh)
{
    if(mesh.empty())
    {
        return false;
    }
    if(mesh.cols < 1 && mesh.rows < 1)
    {
        return false;
    }
    cv::Size meshGridSize = {mesh.cols - 1, mesh.rows - 1};

    std::vector<cv::Point> polygon_tmp;
    for(int i = 0; i < meshGridSize.height; ++i)
    {
        for(int j = 0; j < meshGridSize.width; ++j)
        {
            extractPolygon4(mesh, polygon_tmp, {j,i});
            // Проверить полигон на валидность, должен быть выпуклым
            if(!isConvexPolygon(polygon_tmp, true, false))
            {
                return false;
            }
        }
    }
    return true;
}
