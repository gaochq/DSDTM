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
    void ProcessNewKeyframe();

    //! Recent MapPoints culling refer to orb-slam2
    void MapPointCulling();

    //! Check the size of Keyframe list
    bool CheckNewFrames();

    //! The main function in local mapping
    void Run();

    //! Check whether the stop menu is choosed
    bool IsStopped();

    void Release();

    void RequestFinish();

    bool IsFinished();

    void RequestStart();
    void RequestPause();
    void RequestStop();


protected:

    bool Stop();

    bool CheckFinish();
    void SetFinish();

protected:

    Map         *mMap;
    KeyFrame    *mCurrentKframe;

    std::list<KeyFrame*>    mlNewKeyFrames;
    std::list<MapPoint*>    mlRecentMapPoints;

    bool        mbRequestFinish;
    bool        mbFinished;

    bool        mbStopped;
    bool        mbRequestStop;
    bool        mbPaused;

    std::mutex mMutexKFlist;
    std::mutex mMutexFinish;

};



}// namespace DSDTM