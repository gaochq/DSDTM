//
// Created by buyi on 17-11-4.
//

#include "MapPoint.h"

namespace DSDTM
{

unsigned long MapPoint::mlNextId = 0;

MapPoint::MapPoint()
{

}

MapPoint::MapPoint(Eigen::Vector3d &_pose, Frame *_frame):
        mPose(_pose), mFirstFrame(_frame->mlId)
{
    mlID = mlNextId++;
}


} // namespace DSDTM
