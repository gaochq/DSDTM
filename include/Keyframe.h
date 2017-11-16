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

    //! Add MapPoint and the observation between mappoint and features
    void Add_MapPoint(MapPoint *tMPoint);
    void Add_Observations(long int tId, MapPoint *tMPoint);

    //! Return Observations
    std::map<long int, MapPoint*> Get_Observations() const { return mpObservation;};

public:
    unsigned long           mlId;
    static unsigned long    mlNextId;

protected:
    Frame                   *mFrame;
    std::vector<MapPoint*>  mvMapPoints;
    std::map<long int, MapPoint*>  mpObservation;

};

} //DSDTM



#endif //DSDTM_KEYFRAME_H
