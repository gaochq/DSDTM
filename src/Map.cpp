//
// Created by buyi on 17-11-9.
//

#include "Map.h"


namespace DSDTM
{

Map::Map()
{

}

Map::~Map()
{

}

void Map::AddKeyFrame(KeyFrame *_frame)
{
    msKeyFrames.insert(_frame);
}

void Map::AddMapPoint(MapPoint *_point)
{
    msMapPoints.insert(_point);
}

void Map::GetCloseKeyFrames(const Frame *tFrame, std::list<std::pair<KeyFrame *, double> > &tClose_kfs) const
{
    //TODO improve the stragedy to choose the local keyframes

    for (auto kf : msKeyFrames)
    {
        for (auto keypoint : kf->mvMapPoints)
        {
            if(keypoint->Get_Pose().isZero(0))
                continue;

            if(tFrame->isVisible(keypoint->Get_Pose()))
            {
                tClose_kfs.push_back(
                        std::make_pair(kf, (tFrame->Get_Pose().translation() - kf->Get_Pose().translation()).norm()));
                break;
            }
        }
    }
}

KeyFrame* Map::Get_InitialKFrame()
{
    return *msKeyFrames.begin();
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    return std::vector<MapPoint*>(msMapPoints.begin(), msMapPoints.end());
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    return std::vector<KeyFrame*>(msKeyFrames.begin(), msKeyFrames.end());
}

void Map::Release()
{
    msMapPoints.clear();
    msKeyFrames.clear();
}

}