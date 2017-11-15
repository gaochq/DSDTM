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

    KeyFrame* Map::Get_InitialKFrame()
    {
        return *msKeyFrames.begin();
    }
}