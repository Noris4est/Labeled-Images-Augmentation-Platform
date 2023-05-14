#ifndef WARP_MESH_APPLICATOR
#define WARP_MESH_APPLICATOR

#include <opencv2/opencv.hpp>
#include "marked_frame.hpp"

class MeshWarpApplicator
{
public:
    MeshWarpApplicator();
    MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, bool instantPreprocessing = true);
    MeshWarpApplicator(const cv::Mat &srcMesh, const cv::Mat &dstMesh, cv::Size framesize, cv::Size meshGridSize, bool instantPreprocessing = true);
    void setSrcDstMesh(const cv::Mat &srcMesh, const cv::Mat &dstMesh, bool instantPreprocessing = true);

    cv::Size getMeshGridSize();
    const cv::Mat& getWarpMesh();
    cv::Point apply(cv::Point) const;
    std::vector<cv::Point> apply(const std::vector<cv::Point> &src) const;
    void apply(const cv::Mat &src, cv::Mat &dst) const;
    void apply(const MarkedFrame &src, MarkedFrame &dst) const;
    static bool checkValidMesh(const cv::Mat mesh);
private:
    void preprocessing(); // Вычислительно затратная процедура, формирует map_x, map_y; Для заданного искажения выполняется один раз.
    void perprocessing3nodes();
    cv::Mat mapPolygonIds; // карта id полигонов для определения id полигона попадания любой точки за O(1);
    cv::Mat map_x_inverse; // for remap
    cv::Mat map_y_inverse;
    cv::Mat map_x_direct;
    cv::Mat map_y_direct;
    cv::Mat srcMesh;
    cv::Mat dstMesh;
    cv::Size framesize;
    cv::Size meshGridSize;
    cv::Size dst_frame_size; //TODO: в чем разница между framesize b dst_frame_size;
    bool is_preprocessed = false;
};



# endif // WARP_MESH_APPLICATOR