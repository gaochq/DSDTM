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
        mPose(_pose), mFirstFrameId(_frame->mlId), mnFound(0),
        mlLocalBAKFId(0), mMap(tMap), mObsNum(0), mRefKframe(_frame),
        mnVisible(0)
{
    {
        std::unique_lock<std::mutex> lock(mMap->mMutexMPCreation);
        mlID = mlNextId++;
    }
    mbOutlier = false;
    mdStaticWeight = 1.0;
}

void MapPoint::Set_Pose(Eigen::Vector3d tPose)
{
    std::unique_lock<std::mutex> lock(mMutexPos);

    mPose = tPose;
}

Eigen::Vector3d MapPoint::Get_Pose() const
{
    std::unique_lock<std::mutex> lock(mMutexPos);

    return mPose;
}

void MapPoint::Add_Observation(KeyFrame *tKFrame, size_t tFID)
{
    std::unique_lock<std::mutex> lock(mMutexObs);

    if(mObservations.count(tKFrame))
        return;
    else
        mObservations[tKFrame] = tFID;

    mObsNum++;
}

void MapPoint::Erase_Observation(KeyFrame *tKFrame)
{
    bool bBad = false;
    {
        std::unique_lock<std::mutex> lock(mMutexObs);

        if (mObservations.count(tKFrame))
        {
            mObsNum--;

            mObservations.erase(tKFrame);

            if (tKFrame == mRefKframe) {
                mRefKframe = mObservations.begin()->first;
                mFirstFrameId = mRefKframe->mlId;
            }

            if (mObsNum <= 1)
                bBad = true;
        }
    }

    if(bBad)
        SetBadFlag();

}

std::map<KeyFrame*, size_t> MapPoint::Get_Observations()
{
    std::unique_lock<std::mutex> lock(mMutexObs);

    return mObservations;
}

void MapPoint::SetBadFlag()
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    std::unique_lock<std::mutex> lock2(mMutexPos);

    mbOutlier = true;

    std::map<KeyFrame*, size_t> obs = mObservations;
    mObservations.clear();

    for (auto &iter : obs)
    {
        KeyFrame *tKf = iter.first;
        tKf->Erase_MapPointMatch(iter.second);
    }

    LOG(INFO)<< "MapPoint " << this->mlID << " deleted" <<std::endl;
    mMap->EraseMapPoint(this);
}

bool MapPoint::IsBad() const
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    std::unique_lock<std::mutex> lock2(mMutexPos);

    return mbOutlier;
}

int MapPoint::Get_ObserveNums() const
{
    std::unique_lock<std::mutex> lock(mMutexObs);

    return mObsNum;
}

int MapPoint::Get_FoundNums() const
{
    std::unique_lock<std::mutex> lock(mMutexFounds);

    return mnFound;
}

bool MapPoint::Get_ClosetObs(const Frame *tFrame, Feature *&tFeature, KeyFrame *&tKframe) const
{
    Eigen::Vector3d tPt_frame, tPose;
    std::map<KeyFrame*, size_t> tObservations;
    {
        std::unique_lock<std::mutex> lock2(mMutexObs);
        std::unique_lock<std::mutex> lock(mMutexPos);

        tPose =  mPose;
        tObservations = mObservations;
    }

    tPt_frame = tFrame->Get_CameraCnt() - tPose;
    tPt_frame.normalize();

    double tMin_angle = 0;
    auto min_it = tObservations.begin();
    for (auto iter = tObservations.begin(); iter != tObservations.end(); iter++)
    {
        Eigen::Vector3d tPt_refer = iter->first->Get_CameraCnt() - tPose;
        tPt_refer.normalize();

        double tCos_angle = tPt_refer.dot(tPt_frame);
        if(tCos_angle > tMin_angle)
        {
            tMin_angle = tCos_angle;
            min_it = iter;
        }
    }

    {
        std::unique_lock<std::mutex> lock2(mMutexObs);
        tFeature = min_it->first->mvFeatures[min_it->second];
        tKframe = min_it->first;
    }

    if(tMin_angle < 0.5)
        return false;

    return true;
}

void MapPoint::IncreaseFound(int n)
{
    std::unique_lock<std::mutex> lock(mMutexFounds);

    mnFound = mnFound + n;
}

void MapPoint::EraseFound(int n)
{
    std::unique_lock<std::mutex> lock(mMutexFounds);
    bool tbBad = false;
    {
        //std::unique_lock<std::mutex> lock(mMutexFounds);

        mnFound = mnFound - n;
        if (mnFound <= 0)
            tbBad = true;
    }

    if(tbBad)
        SetBadFlag();
}

void MapPoint::IncreaseVisible(int n)
{
    std::unique_lock<std::mutex> lock(mMutexObs);

    mnVisible += n;
}

int MapPoint::Get_IndexInKeyFrame(KeyFrame *tKf)
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    std::unique_lock<std::mutex> lock2(mMutexPos);

    if(mObservations.count(tKf))
        return mObservations[tKf];
    else
        return -1;
}

void MapPoint::UpdateNormalAndDepth()
{
    std::map<KeyFrame*, size_t> tObservations;
    Eigen::Vector3d tPose;
    KeyFrame *tRefKframe;

    {
        std::unique_lock<std::mutex> lock(mMutexObs);
        std::unique_lock<std::mutex> lock2(mMutexPos);
        tObservations = mObservations;
        tPose = mPose;
        tRefKframe = mRefKframe;
    }

    if(tObservations.empty())
        return;

    Eigen::Vector3d normal = Eigen::MatrixXd::Zero(3,1);
    int n = 0;
    for(const auto &it : tObservations)
    {
        Eigen::Vector3d tVec_C2P = tPose - it.first->Get_CameraCnt();
        tVec_C2P.normalize();

        normal = normal + tVec_C2P;
        n++;
    }

    Eigen::Vector3d tVec_P2Ref = tPose - tRefKframe->Get_CameraCnt();
    const float dist = tVec_P2Ref.norm();

    const int level = tObservations[tRefKframe];
    const float tLevelFactor = 1<<level;
    const int tLevels = 4;
    const float tMinLevelFactor = 1.0/(1<<tLevels);

    {
        std::unique_lock<std::mutex> lock2(mMutexPos);

        mfMaxDistance = dist*tLevelFactor;
        mfMinDistance = mfMaxDistance*tMinLevelFactor;

        mNoramlVector = normal/n;
    }
}

float MapPoint::Get_MaxObserveDistance()
{
    std::unique_lock<std::mutex> lock2(mMutexPos);

    return mfMaxDistance*1.2;
}

float MapPoint::Get_MinObserveDistance()
{
    std::unique_lock<std::mutex> lock2(mMutexPos);

    return mfMinDistance*0.8;
}

Eigen::Vector3d MapPoint::Get_NormalVector()
{
    std::unique_lock<std::mutex> lock2(mMutexPos);

    return mNoramlVector;
}

float MapPoint::Get_FoundRatio()
{
    std::unique_lock<std::mutex> lock(mMutexObs);
    std::unique_lock<std::mutex> lock2(mMutexFounds);

    return (1.0*mnFound/mnVisible);
}

} // namespace DSDTM
