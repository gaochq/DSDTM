//
// Created by buyi on 17-11-9.
//
#include "Keyframe.h"


namespace DSDTM
{
long unsigned int KeyFrame::mlNextId=0;
KeyFrame::KeyFrame(const FramePtr tframe):
        mFrame(tframe), mvMapPoints(tframe->mvMapPoints), mTimeStamp(tframe->mdCloTimestamp),
        mvFeatures(tframe->mvFeatures), mnVaildMps(0), mCamera(tframe->mCamera), mRefId(tframe->mlId),
        mClolorImg(tframe->mColorImg), mDepthImg(tframe->mDepthImg), mvImg_Pyr(tframe->mvImg_Pyr)
{
    mlId = mlNextId++;
    mlFixedLocalBAKfId = mlId;
    mlLocalBAKfId = mlId;

    mvMapPoints.resize(mFrame->mvFeatures.size());
    Set_Pose(tframe->Get_Pose());

    std::for_each(tframe->mpObservation.begin(), tframe->mpObservation.end(), [&](std::pair<size_t , MapPoint*> tObs)
    {
        Add_Observations(tObs.first, tObs.second);
    });
}

KeyFrame::~KeyFrame()
{

}

void KeyFrame::Add_MapPoint(MapPoint *tMPoint, int tIdx)
{
    mvMapPoints[tIdx] = tMPoint;
    mnVaildMps++;
}

void KeyFrame::Add_Observations(size_t tId, MapPoint *tMPoint)
{
    mpFFObservation.insert(std::pair<long int, size_t>(mvFeatures[tId]->mlId, tId));
    mpObservation.insert(std::pair<long int, MapPoint*>(mvFeatures[tId]->mlId, tMPoint));
}

std::vector<MapPoint*> KeyFrame::GetMapPoints()
{
    return mvMapPoints;
}

void KeyFrame::Set_Pose(Sophus::SE3 tPose)
{
    mT_c2w = tPose;

    Sophus::SE3 tT_w2c = mT_c2w.inverse();
    mR_w2c = tT_w2c.so3();
    mOw = tT_w2c.translation();
}

Eigen::Vector2d KeyFrame::World2Pixel(const Eigen::Vector3d &Point)
{
    Eigen::Vector3d tPoint = mT_c2w*Point;

    return mCamera->Camera2Pixel(tPoint);
}

void KeyFrame::UpdateConnection()
{
    std::map<KeyFrame*, int> tKFLocalMap;

    //TODO add lock
    std::vector<MapPoint*> tMptSets = mvMapPoints;

    for (int i = 0; i < tMptSets.size(); ++i)
    {
        MapPoint *tMp = tMptSets[i];

        if(!tMp)
            continue;

        if(tMp->IsBad())
            continue;

        std::map<KeyFrame*, size_t> tMpObservations = tMp->Get_Observations();
        for (const auto &iter : tMpObservations)
        {
            if(iter.first->mlId == mlId)
                continue;

            tKFLocalMap[iter.first]++;
        }
    }

    if(tKFLocalMap.empty())
        return;

    std::vector<std::pair<int, KeyFrame*> > tCovKFPairss;

    int threshold = 15;
    int tMaxCount = 0;
    KeyFrame *tMaxKF = NULL;

    for (const auto &iter : tKFLocalMap)
    {
        if(iter.second > tMaxCount)
        {
            tMaxCount = iter.second;
            tMaxKF = iter.first;
        }

        if(iter.second > threshold)
        {
            //TODO update the connection of another KF
            tCovKFPairss.push_back(std::make_pair(iter.second, iter.first));
        }
    }

    if(tCovKFPairss.empty())
    {
        tCovKFPairss.push_back(std::make_pair(tMaxCount, tMaxKF));

        //TODO update the connection of tMaxKF
    }

    std::sort(tCovKFPairss.begin(), tCovKFPairss.end());

    mOrderedCovGraph = tCovKFPairss;
    mConnectedKFs = tKFLocalMap;
}

void KeyFrame::AddConnection(KeyFrame *tKF, int tWeight)
{
    mConnectedKFs[tKF] = tWeight;

    UpdateCovGraph(tKF, tWeight);
}

void KeyFrame::UpdateCovGraph(KeyFrame *tKF, int tWeight)
{
    auto it = std::find_if(mOrderedCovGraph.begin(), mOrderedCovGraph.end(),
                           [&](const std::pair<int, KeyFrame*>& element){ return element.second == tKF; });

    if(it!=mOrderedCovGraph.end())
        it->first = tWeight;
    else
        mOrderedCovGraph.push_back(std::make_pair(tWeight, tKF));

    std::sort(mOrderedCovGraph.begin(), mOrderedCovGraph.end());

}

std::vector<std::pair<int, KeyFrame*>> KeyFrame::GetCovKFrames()
{
    //TODO add lock

    return mOrderedCovGraph;
}

} //namesapce DSDTM