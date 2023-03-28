#pragma once 

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "draw_custom_line.hpp"

void getAllPolygonPoints(const std::vector<cv::Point2f> &polygon, std::vector<cv::Point2f> &includedPoints);

void getPoly4(const cv::Mat &mesh_src, std::vector<cv::Point2f> &poly_dst, cv::Point topLeftCorner);

/*
Генерация двух 3х вершинных полигонов из одного 4х вершинного.
При mainDiagonalSeparate=true используется разделяющая диагональ
от левого верхнего до правого нижнего угла 
(главная в терминах линейной алгебры). 
Если false, то используется побочная диагональ.
ВАЖНО! При учете, что вершины исходном полигоне по часовой стрелке
*/
void getPair3nodesPolyFrom4nodesPoly(
    const std::vector<cv::Point2f> &poly_4n_src, 
    std::vector<std::vector<cv::Point2f>> &array_poly_3n_dst,
    bool mainDiagonalSeparate = true);

cv::Size getMeshSize(const cv::Mat &mesh);

bool checkAndAdaptingPointInFrame(cv::Point &src_p, cv::Size framesize);

void drawMeshTransform(
    cv::Mat &frame, 
    const cv::Mat &origMesh, 
    const cv::Mat &warpMesh, 
    cv::Scalar origNodesColor = {0,255,0}, 
    cv::Scalar warpNodesColor = {0,0,255}, 
    cv::Scalar vertexColor = {0,0,0},
    int origNodesRadius = 3,
    int warpNodesRadius = 3);

void drawMesh(cv::Mat &frame, const cv::Mat &mesh, cv::Scalar color, bool draw_edges = false, int linestyle = cv::LineStyles::SOLID);
void drawMesh3nodes(cv::Mat &frame, const cv::Mat &mesh, cv::Scalar color, bool draw_edges = false, int linestyle = cv::LineStyles::SOLID);

void convertfromAffine2perspectiveMatrix(const cv::Mat &src_affineMat, cv::Mat &dst_perspectiveMat);

void convertIntPolygon2Float(const std::vector<cv::Point2i> &srcPoly2i, std::vector<cv::Point2f> &dstPoly2f);

void extractPolygon4(const cv::Mat &mesh, std::vector<cv::Point> &polygon, cv::Point topleft);
void extractPolygon3(const cv::Mat &mesh, std::vector<cv::Point> &polygon, cv::Point pos);

std::vector<cv::Point2f> convertIntPolygon2Float(const std::vector<cv::Point2i> &srcPoly2i);