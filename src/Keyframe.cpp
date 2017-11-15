//
// Created by buyi on 17-11-9.
//
#include "Keyframe.h"


namespace DSDTM
{
    KeyFrame::KeyFrame(Frame &_frame):
            mFrame(&_frame)
    {

    }

    KeyFrame::~KeyFrame()
    {

    }

    void KeyFrame::Add_MapPoint(MapPoint *tMPoint)
    {
        mvMapPoints.push_back(tMPoint);
    }


} //namesapce DSDTM