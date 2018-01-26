//
// Created by buyi on 18-1-22.
//

#ifndef DSDTM_UTILS_H
#define DSDTM_UTILS_H

#include "Camera.h"

namespace DSDTM
{
namespace utils
{

inline Eigen::Vector2d Normalizezation2D(const Eigen::Vector3d &tMapPoint)
{
    return tMapPoint.head<2>()/tMapPoint[2];
}

inline double ReprojectionError(Eigen::Vector3d tPixNormal, const Sophus::SE3 &tKfPose, const Eigen::Vector3d &tMapPoint)
{
    Eigen::Vector3d tCamPt = tKfPose*tMapPoint;

    Eigen::Vector2d tReidual = Normalizezation2D(tPixNormal) - Normalizezation2D(tCamPt);

    return tReidual.dot(tReidual);
}

inline double MedianAbsDev(const std::vector<double> &tResiduals)
{

}

inline double GetMedian(std::vector<double> tdVector)
{
    std::vector<double>::iterator iter = tdVector.begin() + floor(tdVector.size()/2);
    std::nth_element(tdVector.begin(), iter, tdVector.end());

    return *iter;
}
} // UTILS

} // DSDTM



#endif //DSDTM_UTILS_H
