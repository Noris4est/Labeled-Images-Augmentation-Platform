#ifndef WARP_MESH_APPLICATOR
#define WARP_MESH_APPLICATOR

#include <opencv2/opencv.hpp>

class MeshWarpApplicator
{
public:
    MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, bool instantPreprocessing = true);
    MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, cv::Size framesize, cv::Size meshGridSize, bool instantPreprocessing = true);
    cv::Point apply(cv::Point) const;
    std::vector<cv::Point> apply(const std::vector<cv::Point> &src) const;
    void apply(const cv::Mat &src, cv::Mat &dst) const;
private:
    void preprocessing(); // Вычислительно затратная процедура, формирует map_x, map_y; Для заданного искажения выполняется один раз.
    void perprocessing3nodes();
    cv::Mat mapPolygonIds; // карта id полигонов для определения id полигона попадания любой точки за O(1);
    cv::Mat map_x; // for remap
    cv::Mat map_y;
    cv::Mat srcMesh;
    cv::Mat dstMesh;
    cv::Size framesize;
    cv::Size meshGridSize;
};



# endif // WARP_MESH_APPLICATOR