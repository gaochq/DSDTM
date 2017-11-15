//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_KEYFRAME_H
#define DSDTM_KEYFRAME_H

#include "Camera.h"
#include "Frame.h"
#include "MapPoint.h"

namespace DSDTM
{

class MapPoint;

class KeyFrame
{
public:
    KeyFrame(Frame &_frame);
    ~KeyFrame();

    void Add_MapPoint(MapPoint *tMPoint);

public:
    unsigned long           mlId;
    static unsigned long    mlNextId;

protected:
    Frame                   *mFrame;
    std::vector<MapPoint*>  mvMapPoints;

};

} //DSDTM



#endif //DSDTM_KEYFRAME_H
