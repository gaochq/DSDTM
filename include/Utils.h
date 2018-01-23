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

    return tReidual.norm();
}


} // UTILS

} // DSDTM



#endif //DSDTM_UTILS_H
