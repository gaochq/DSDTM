//
// Created by buyi on 17-11-4.
//

#ifndef DSDTM_MAPPOINT_H
#define DSDTM_MAPPOINT_H

#include "Camera.h"
#include "Frame.h"
#include "Feature.h"


namespace DSDTM
{

class Frame;

class MapPoint
{
public:
    MapPoint();
    MapPoint(Eigen::Vector3d& _pose, Frame *_frame);

public:
    unsigned long           mlID;
    static unsigned long    mlNextId;
    Eigen::Vector3d         mPose;

private:

    unsigned long           mFirstFrame;
    std::list<Feature>      mlObs;
};


} // namesapce DSDTM




#endif //DSDTM_MAPPOINT_H
