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
    std::unique_lock<std::mutex> lock(mMapMutex);
    msKeyFrames.insert(_frame);
}

void Map::AddMapPoint(MapPoint *_point)
{
    std::unique_lock<std::mutex> lock(mMapMutex);
    msMapPoints.insert(_point);
}

KeyFrame* Map::Get_InitialKFrame()
{
    return *msKeyFrames.begin();
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMapMutex);
    return std::vector<MapPoint*>(msMapPoints.begin(), msMapPoints.end());
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMapMutex);
    return std::vector<KeyFrame*>(msKeyFrames.begin(), msKeyFrames.end());
}

int Map::ReturnKeyFramesSize()
{
    return msKeyFrames.size();
}

void Map::EraseMapPoint(MapPoint *tMp)
{
    //TODO add lock
    msMapPoints.erase(tMp);
}

void Map::Release()
{
    msMapPoints.clear();
    msKeyFrames.clear();
}

}// namespace DSDTM