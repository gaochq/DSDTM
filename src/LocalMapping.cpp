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

bool LocalMapping::CheckNewFrames()
{
    //std::unique_lock<std::mutex> lock(mMutexKFlist);

    return mlNewKeyFrames.empty();
}

void LocalMapping::Run()
{


}

}//namespace DSDTM