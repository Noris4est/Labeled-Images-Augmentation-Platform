
#include "utils.hpp"
#include "draw_custom_line.hpp"

void getAllPolygonPoints(const std::vector<cv::Point2f> &polygon, std::vector<cv::Point2f> &includedPoints)
{
    assert(cv::contourArea(polygon) != 0);
    includedPoints.clear();
    includedPoints.shrink_to_fit();
    //Определение размеров tmp_cv_Mat под маску
    int x0 = polygon[0].x, y0 = polygon[0].y;
    int min_i = y0, min_j = x0, max_i = y0, max_j = x0;
    for(const auto &p : polygon)
    {
        if(p.x > max_j)
            max_j = p.x;
        if(p.x < min_j)
            min_j = p.x;
        if(p.y > max_i)
            max_i = p.y;
        if(p.y < min_i)
            min_i = p.y;
    }
    x0 = min_j;
    y0 = min_i;
    int offset = 3;
    int frame_w = max_j - min_j + 2*offset;
    int frame_h = max_i - min_i + 2*offset;
    cv::Mat frame(cv::Size(frame_w, frame_h), CV_8UC1);
    includedPoints.reserve(frame_w * frame_h);
    frame = cv::Scalar(0);
    std::vector<cv::Point2i> poly = {};
    poly.reserve(polygon.size());
    cv::Point2i tmp_p;
    for(int i = 0; i < polygon.size(); ++i)
    {
        tmp_p = (cv::Point2i)polygon[i];
        tmp_p.x += -x0 + offset;
        tmp_p.y += -y0 + offset;
        poly.push_back(tmp_p);
    }
    std::vector<std::vector<cv::Point2i>> poly_array = {poly};
    cv::fillPoly(frame, poly_array, cv::Scalar(255), cv::LineTypes::LINE_4);
    int x_tmp, y_tmp;
    for(int i = 0; i < frame.rows; i++)
    {
        for(int j = 0; j < frame.cols; j++)
        {
            if(frame.at<uint8_t>(i, j) != 0)
            {
                x_tmp = j - offset + x0;
                if(x_tmp < 0)
                    x_tmp = 0;
                y_tmp = i - offset + y0;
                if(y_tmp < 0)
                    y_tmp = 0;
                includedPoints.push_back(cv::Point2f(x_tmp, y_tmp));
            }
        }
    }
}

void getPoly4(const cv::Mat &mesh_src, std::vector<cv::Point2f> &poly_dst, cv::Point topLeftCorner)
{
    int i0 = topLeftCorner.y;
    int j0 = topLeftCorner.x;
    poly_dst.clear();
    poly_dst.shrink_to_fit();
    poly_dst.reserve(4);
    //clockwise
    cv::Point2i tl = mesh_src.at<cv::Point2i>(i0, j0); //top left
    cv::Point2i tr = mesh_src.at<cv::Point2i>(i0, j0 + 1); //top right
    cv::Point2i br = mesh_src.at<cv::Point2i>(i0 + 1, j0 + 1); //bottom right
    cv::Point2i bl = mesh_src.at<cv::Point2i>(i0 + 1, j0); // bottom left
    poly_dst = {tl, tr, br, bl};
}

void getPair3nodesPolyFrom4nodesPoly(
    const std::vector<cv::Point2f> &poly_4n_src, 
    std::vector<std::vector<cv::Point2f>> &array_poly_3n_dst,
    bool mainDiagonalSeparate)
{
    std::vector<cv::Point2f> poly3n_1, poly3n_2;
    if(mainDiagonalSeparate)
    {
        poly3n_1 = {poly_4n_src[0], poly_4n_src[1], poly_4n_src[2]};
        poly3n_2 = {poly_4n_src[0], poly_4n_src[2], poly_4n_src[3]};
    }
    else
    {
        poly3n_1 = {poly_4n_src[0], poly_4n_src[1], poly_4n_src[3]};
        poly3n_2 = {poly_4n_src[3], poly_4n_src[1], poly_4n_src[2]};
    }
    array_poly_3n_dst = {poly3n_1, poly3n_2};
}

cv::Size getMeshSize(const cv::Mat &mesh)
{
    int max_x = 0, max_y = 0;
    for(int i = 0; i < mesh.rows; ++i)
    {
        for(int j = 0; j < mesh.cols; ++j)
        {
            auto p = mesh.at<cv::Point2i>(i, j);
            if(p.x > max_x)
                max_x = p.x;
            if(p.y > max_y)
                max_y = p.y;
        }
    }
    return {max_x, max_y};
}
bool checkAndAdaptingPointInFrame(cv::Point &src_p, cv::Size framesize)
{
    float tolerance = 0.1;
    if(src_p.x > 0 && src_p.y > 0 && src_p.x < framesize.width && src_p.y < framesize.height)
    {
        return true;
    }
    else
    {
        bool cond1 = true, cond2 = true, cond3 = true, cond4 = true;
        if(src_p.x < 0)
        {
            if(src_p.x > - tolerance * framesize.width)
            {
                src_p.x = 0;
            }
            else
            {
                cond1 = false;
            }
        }

        if(src_p.y < 0)
        {
            if(src_p.y > tolerance * framesize.height)
            {
                src_p.y = 0;
            }
            else
            {
                cond2 = false;
            }
        }

        if(src_p.x >= framesize.width)
        {
            if(src_p.x < (1 + tolerance) * framesize.width)
            {
                src_p.x = framesize.width - 1;
            }
            else
            {
                cond3 = false;
            }
        }
        
        if(src_p.y >= framesize.height)
        {
            if(src_p.y < (1 + tolerance) * framesize.height)
            {
                src_p.y = framesize.height - 1;
            }
            else
            {
                cond4 = false;
            }
        }
        return cond1 && cond2 && cond3 && cond4;
    }
}

void drawMeshTransform(
    cv::Mat &frame, 
    const cv::Mat &origMesh, 
    const cv::Mat &warpMesh, 
    cv::Scalar origNodesColor, 
    cv::Scalar warpNodesColor, 
    cv::Scalar vertexColor,
    int origNodesRadius,
    int warpNodesRadius)
{
    assert(!origMesh.empty() && !warpMesh.empty() && origMesh.size() == warpMesh.size());

    cv::Point2i p_orig, p_warp;
    for(int i = 0; i < origMesh.rows; ++i)
    {
        for(int j = 0; j < warpMesh.cols; ++j)
        {
            p_orig = origMesh.at<cv::Point2i>(i, j);
            p_warp = warpMesh.at<cv::Point2i>(i, j);
            // cv::line(frame, p_orig, p_warp, vertexColor, 1, cv::LINE_AA);
            cv::circle(frame, p_warp, warpNodesRadius, warpNodesColor, -1, cv::LINE_AA);
            cv::arrowedLine(frame, p_orig, p_warp, vertexColor, 1, cv::LINE_AA, 0, 0.4);
            cv::circle(frame, p_orig, origNodesRadius, origNodesColor, 1, cv::LINE_AA);

        }
    }
}
void drawMesh(cv::Mat &frame, const cv::Mat &mesh, cv::Scalar color, bool draw_edges, int linestyle)
{
    int radius = 4;

    cv::Point tmp_p;
    for(int i = 0; i < mesh.rows; ++i)
    {
        for(int j = 0; j < mesh.cols; ++j)
        {
            tmp_p = mesh.at<cv::Point2i>(i,j);
            cv::circle(frame, tmp_p, radius, color, -1);
        }
    }

    if(draw_edges)
    {
        cv::Point2i tmp_cur, pRight, pBottom;
        for(int i = 0; i < mesh.rows - 1; ++i)
        {
            for(int j = 0; j < mesh.cols - 1; ++j)
            {
                tmp_cur = mesh.at<cv::Point2i>(i,j);
                pRight = mesh.at<cv::Point2i>(i, j+1);
                pBottom = mesh.at<cv::Point2i>(i+1, j);
                cv::line_custom(frame, tmp_cur, pRight, color, 1, cv::LINE_AA, linestyle);
                cv::line_custom(frame, tmp_cur, pBottom, color, 1, cv::LINE_AA, linestyle);
            }
        }   
    }
}

void drawMesh3nodes(cv::Mat &frame, const cv::Mat &mesh, cv::Scalar color, bool draw_edges, int linestyle)
{
    int meshGridW = (mesh.cols - 2) * 2 + 1;
    int meshGridH = mesh.rows - 1;

    int radius = 4;

    cv::Point tmp_p;
    for(int i = 0; i < meshGridH; ++i)
    {
        for(int j = 0; j < meshGridW; ++j)
        {
            tmp_p = mesh.at<cv::Point2i>(i,j);
            cv::circle(frame, tmp_p, radius, color, -1);
        }
    }

    if(draw_edges)
    {
        std::vector<cv::Point> tmp_poly;
        cv::Point p1, p2, p3;
        cv::Point2i tmp_cur, pRight, pBottom;
        for(int i = 0; i < meshGridH; ++i)
        {
            for(int j = 0; j < meshGridW; ++j)
            {
                extractPolygon3(mesh, tmp_poly, {j, i});
                p1 = tmp_poly[0]; p2 = tmp_poly[1]; p3 = tmp_poly[2];
                cv::line_custom(frame, p1, p2, color, 1, cv::LINE_AA, linestyle);
                cv::line_custom(frame, p2, p3, color, 1, cv::LINE_AA, linestyle);
                cv::line_custom(frame, p1, p3, color, 1, cv::LINE_AA, linestyle);
            }
        }   
    }
}

void convertfromAffine2perspectiveMatrix(const cv::Mat &src_affineMat, cv::Mat &dst_perspectiveMat)
{
    dst_perspectiveMat = cv::Mat::eye(cv::Size(3,3), src_affineMat.type());
    src_affineMat.copyTo(dst_perspectiveMat(cv::Rect(0,0,3,2)));
}

void convertIntPolygon2Float(const std::vector<cv::Point2i> &srcPoly2i, std::vector<cv::Point2f> &dstPoly2f)
{
    dstPoly2f.clear();
    dstPoly2f.shrink_to_fit();
    dstPoly2f.reserve(srcPoly2i.size());
    for(const auto &el : srcPoly2i)
    {
        dstPoly2f.push_back(static_cast<cv::Point2f>(el));
    }
}

std::vector<cv::Point2f> convertIntPolygon2Float(const std::vector<cv::Point2i> &srcPoly2i)
{
    std::vector<cv::Point2f> dst;
    convertIntPolygon2Float(srcPoly2i, dst);
    return dst;
}

void extractPolygon4(const cv::Mat &mesh, std::vector<cv::Point> &polygon, cv::Point topleft)
{
    int i0 = topleft.y;
    int j0 = topleft.x;
    polygon.clear();
    polygon.shrink_to_fit();
    polygon.reserve(4);
    //clockwise
    cv::Point2i tl = mesh.at<cv::Point2i>(i0, j0); //top left
    cv::Point2i tr = mesh.at<cv::Point2i>(i0, j0 + 1); //top right
    cv::Point2i br = mesh.at<cv::Point2i>(i0 + 1, j0 + 1); //bottom right
    cv::Point2i bl = mesh.at<cv::Point2i>(i0 + 1, j0); // bottom left
    polygon = {tl, tr, br, bl};
}

void extractPolygon3(const cv::Mat &mesh, std::vector<cv::Point> &polygon, cv::Point position)
{
    int meshGridWidth = (mesh.cols - 2) * 2 + 1;
    int meshGridHeight = mesh.rows - 1;
    assert(position.x < meshGridWidth && position.x >= 0 && position.y < meshGridHeight && position.y >= 0);
    int jpos = position.x;
    int ipos = position.y;
    int tmp;
    polygon.clear();
    polygon.shrink_to_fit();
    polygon.reserve(3);
    cv::Point2i p1, p2, p3;//clockwise
    tmp = jpos / 2; // целая часть 
    if(ipos % 2 == 0)
    {
        if(jpos % 2 == 0)
        {
            
            p1 = mesh.at<cv::Point2i>(ipos, tmp); //top left
            p2 = mesh.at<cv::Point2i>(ipos, tmp + 1); // top right
            p3 = mesh.at<cv::Point2i>(ipos + 1, tmp); // bottom center
        }
        else
        {
            p1 = mesh.at<cv::Point2i>(ipos + 1, tmp); //left bottom
            p2 = mesh.at<cv::Point2i>(ipos, tmp + 1); // top center
            p3 = mesh.at<cv::Point2i>(ipos + 1, tmp + 1); // right bottom
        }
    }
    else
    {
        if(jpos % 2 == 0)
        {   
            p1 = mesh.at<cv::Point2i>(ipos + 1, tmp); // bl
            p2 = mesh.at<cv::Point2i>(ipos, tmp); // tl
            p3 = mesh.at<cv::Point2i>(ipos + 1, tmp + 1);//br
        }
        else
        {
            p1 = mesh.at<cv::Point2i>(ipos + 1, tmp + 1); //bottom center
            p2 = mesh.at<cv::Point2i>(ipos, tmp); // tl
            p3 = mesh.at<cv::Point2i>(ipos, tmp + 1); // tr
        }
    }
    polygon = {p1, p2, p3};
}

