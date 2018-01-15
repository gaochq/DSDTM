//
// Created by buyi on 17-11-4.
//

#include "MapPoint.h"

namespace DSDTM
{

unsigned long MapPoint::mlNextId = 0;

MapPoint::MapPoint()
{
    mbOutlier = false;
    mdStaticWeight = 1.0;
}

MapPoint::MapPoint(Eigen::Vector3d &_pose, KeyFrame *_frame):
        mPose(_pose), mFirstFrame(_frame->mlId), mnFound(0)
{
    mlID = mlNextId++;
    mbOutlier = false;
    mdStaticWeight = 1.0;
}

void MapPoint::Set_Pose(Eigen::Vector3d tPose)
{
    mPose = tPose;
}

Eigen::Vector3d MapPoint::Get_Pose() const
{
    return mPose;
}

void MapPoint::Add_Observation(KeyFrame *tKFrame, size_t tFID)
{
    if(mObservations.count(tKFrame))
        return;
    else
        mObservations[tKFrame] = tFID;

    mObsNum++;
}

void MapPoint::SetBadFlag()
{
    mbOutlier = true;
}

bool MapPoint::IsBad() const
{
    return mbOutlier;
}

int MapPoint::Get_ObserveNums() const
{
    return mnFound;
}

bool MapPoint::Get_ClosetObs(const Frame *tFrame, Feature *&tFeature, KeyFrame *&tKframe) const
{
    Eigen::Vector3d tPt_frame = mPose - tFrame->Get_CameraCnt();
    tPt_frame.normalize();

    double tMin_angle = 0;
    for (auto iter = mObservations.begin(); iter != mObservations.end(); iter++)
    {
        Eigen::Vector3d tPt_refer = mPose - iter->first->Get_CameraCnt();
        tPt_refer.normalize();

        double tCos_angle = tPt_refer.dot(tPt_frame);
        if(tCos_angle > tMin_angle)
        {
            tMin_angle = tCos_angle;
            tFeature = iter->first->mvFeatures[iter->second];

            tKframe = iter->first;
        }
    }

    if(tMin_angle < 0.5)
        return false;

    return true;
}

void MapPoint::IncreaseFound(int n)
{
    mnFound = mnFound + n;
}

void MapPoint::SetStaticWeight(double tWeight)
{
    mdStaticWeight = 0.5*mdStaticWeight + 0.5*tWeight;
}

double MapPoint::GetStaticWeight()
{
    return mdStaticWeight;
}



} // namespace DSDTM
