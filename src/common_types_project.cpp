#include "common_types_project.hpp"

QualityScores operator+(const QualityScores & lhs, const QualityScores & rhs)
{
    QualityScores result;
    result.tp = lhs.tp + rhs.tp;
    result.fp = lhs.fp + rhs.fp;
    result.fn = lhs.fn + rhs.fn;
    result.iou = lhs.iou + rhs.iou;
    return result;
}

QualityScores QualityScores::operator*(double value)
{
    return QualityScores{this->tp * value, this->fp * value, this->fn * value, this->iou * value};
}
