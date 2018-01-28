//
// Created by buyi on 17-11-20.
//
#include "LocalMapping.h"


namespace DSDTM
{
LocalMapping::LocalMapping(Map *tMap):
    mMap(tMap)
{

}

void LocalMapping::InsertKeyFrame(KeyFrame *tKFrame)
{
    //std::unique_lock<std::mutex> lock(mMutexKFlist);

    mlNewKeyFrames.push_back(tKFrame);
}

void LocalMapping::ProcessNewKeyframe(KeyFrame *tKf)
{
    //std::unique_lock<std::mutex> lock(mMutexKFlist);
    //mCurrentKframe = mlNewKeyFrames.front();
    //mlNewKeyFrames.pop_front();

    mCurrentKframe = tKf;

    const std::vector<MapPoint*> mvMapPoints = mCurrentKframe->GetMapPoints();
    for (const auto &it : mvMapPoints)
    {
        MapPoint *tMp = it;

        if(tMp)
        {
            if(!(tMp->IsBad()))
            {
                mlRecentMapPoints.push_back(tMp);
            }
        }
    }
}

void LocalMapping::MapPointCulling()
{
    const int tnObsTh = 2;
    const unsigned long int mnCurrentKfId = mCurrentKframe->mlId;

    std::list<MapPoint*>::iterator it = mlRecentMapPoints.begin();
    while(it!=mlRecentMapPoints.end())
    {
        MapPoint *tMp = (*it);

        if(tMp->IsBad())
        {
            it = mlRecentMapPoints.erase(it);
        }
        else if(tMp->Get_FoundRatio() < 0.25)
        {
            tMp->SetBadFlag();
            it = mlRecentMapPoints.erase(it);
        }
        else if((mnCurrentKfId - tMp->mFirstFrameId) >= 2 && (tMp->Get_ObserveNums() < tnObsTh))
        {
            tMp->SetBadFlag();
            it = mlRecentMapPoints.erase(it);
        }
        else if((mnCurrentKfId - tMp->mFirstFrameId) >= 3)
            it = mlRecentMapPoints.erase(it);
        else
            it++;
    }
}

bool LocalMapping::CheckNewFrames()
{
    //std::unique_lock<std::mutex> lock(mMutexKFlist);

    return mlNewKeyFrames.empty();
}

void LocalMapping::Run(KeyFrame *tKf)
{
    //ProcessNewKeyframe(tKf);
    //MapPointCulling();

    mCurrentKframe = tKf;

    if(mMap->ReturnKeyFramesSize() > 2)
        Optimizer::LocalBundleAdjustment(mCurrentKframe, mMap);
}

}//namespace DSDTM