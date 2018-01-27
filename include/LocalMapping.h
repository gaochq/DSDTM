//
// Created by buyi on 17-11-20.
//
#include "Camera.h"
#include "Map.h"
#include "Keyframe.h"
#include "MapPoint.h"
#include "Optimizer.h"

namespace DSDTM
{
class Map;
class KeyFrame;
class MapPoint;

class LocalMapping
{
public:
    LocalMapping(Map *tMap);
    ~LocalMapping();

    //! Insert keyframe into LocalMap
    void InsertKeyFrame(KeyFrame *tKFrame);

    //! Insert keyframe into map and create new MapPoints
    void ProcessNewKeyframe(KeyFrame *tKf= static_cast<KeyFrame*>(NULL));

    //! Recent MapPoints culling refer to orb-slam2
    void MapPointCulling();

    //! Check the size of Keyframe list
    bool CheckNewFrames();

    //! The main function in local mapping
    void Run(KeyFrame *tKf= static_cast<KeyFrame*>(NULL));


protected:

    Map         *mMap;
    KeyFrame    *mCurrentKframe;

    std::list<KeyFrame*>    mlNewKeyFrames;
    std::list<MapPoint*>    mlRecentMapPoints;

    std::mutex mMutexKFlist;

};



}// namespace DSDTM