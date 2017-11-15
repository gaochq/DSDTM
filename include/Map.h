//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_MAP_H
#define DSDTM_MAP_H



#include "Camera.h"
#include "Keyframe.h"
#include "MapPoint.h"
#include "Frame.h"


namespace DSDTM
{

class KeyFrame;
class Frame;
class MapPoint;

class Map
{
public:
    Map();
    ~Map();

    //! Add keyfram and mapPoint
    void AddKeyFrame(KeyFrame *_frame);
    void AddMapPoint(MapPoint *_point);

    KeyFrame *Get_InitialKFrame();


protected:
    std::set<KeyFrame*>     msKeyFrames;
    std::set<MapPoint*>     msMapPoints;

};

} //namespace DSDTM

#endif //DSDTM_MAP_H