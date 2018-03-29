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

    //! Get the initial Frame
    KeyFrame *Get_InitialKFrame();

    //! Get all MapPoints
    std::vector<MapPoint*> GetAllMapPoints();

    //! Get all KeyFrames
    std::vector<KeyFrame*> GetAllKeyFrames();
    int ReturnKeyFramesSize();

    //! Set and get local MapPoints for viewer
    void SetReferenceMapPoints(const std::vector<MapPoint*> tMps);
    std::vector<MapPoint*> GetReferenceMapPoints();

    //! Erase MapPoint
    void EraseMapPoint(MapPoint *tMp);

    //! Clera all vars in map
    void Release();

public:

    //! Avoid simultaneously create mappoint in separate thread
    std::mutex mMutexMPCreation;
    std::mutex mMutexMapUpdate;

protected:
    std::set<KeyFrame*>     msKeyFrames;
    std::set<MapPoint*>     msMapPoints;

    std::vector<MapPoint*>  mvpReferenceMapPoints;

    std::mutex mMapMutex;
};

} //namespace DSDTM

#endif //DSDTM_MAP_H