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

MapPoint::MapPoint(Eigen::Vector3d &_pose, KeyFrame *_frame):
        mPose(_pose), mFirstFrame(_frame->mlId)
{
    mlID = mlNextId++;
}

void MapPoint::Set_Pose(Eigen::Vector3d tPose)
{
    mPose = tPose;
}

Eigen::Vector3d MapPoint::Get_Pose()
{
    return mPose;
}

void MapPoint::Add_Observation(KeyFrame *tKFrame, size_t tFID)
{
    if(mObservations.count(tKFrame))
        return;
    else
        mObservations[tKFrame] = tFID;

    mObserveNum++;
}


} // namespace DSDTM
