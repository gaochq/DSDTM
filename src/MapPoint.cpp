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

MapPoint::MapPoint(Eigen::Vector3d &_pose, KeyFrame *_frame, Map *tMap):
        mPose(_pose), mFirstFrame(_frame->mlId), mnFound(0),
        mlLocalBAKFId(0), mMap(tMap), mObsNum(0)
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
    //!TODO add lock

    if(mObservations.count(tKFrame))
        return;
    else
        mObservations[tKFrame] = tFID;

    mObsNum++;
}

void MapPoint::Erase_Observation(KeyFrame *tKFrame)
{
    bool bBad = false;

    if(mObservations.count(tKFrame))
    {
        mObsNum--;

        mObservations.erase(tKFrame);

        if(mObsNum <=1)
            bBad = true;
    }

    if(bBad)
        SetBadFlag();

}

std::map<KeyFrame*, size_t> MapPoint::Get_Observations()
{
    //TODO add lock
    return mObservations;
}

void MapPoint::SetBadFlag()
{
    mbOutlier = true;

    std::map<KeyFrame*, size_t> obs = mObservations;
    mObservations.clear();

    for (auto &iter : obs)
    {
        KeyFrame *tKf = iter.first;
        tKf->Erase_MapPointMatch(iter.second);
    }

    mMap->EraseMapPoint(this);
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

int MapPoint::Get_IndexInKeyFrame(KeyFrame *tKf)
{
    if(mObservations.count(tKf))
        return mObservations[tKf];
    else
        return -1;
}



} // namespace DSDTM
