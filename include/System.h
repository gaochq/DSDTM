//
// Created by buyi on 17-11-22.
//

#ifndef DSDTM_SYSTEM_H_H
#define DSDTM_SYSTEM_H_H

#include "Camera.h"
#include "Tracking.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Viewer.h"

namespace DSDTM
{

class Tracking;
class Viewer;
class MapDrawer;

class System
{

public:

    System(const std::string &Paramfile, const Camera_Model tSensor, const bool tbUseViewer = true);

    Sophus::SE3 TrackRGBD(cv::Mat &tColorImg, cv::Mat &tDepthImg, const double &timestamp);

    Sophus::SE3 TrackMonocular(cv::Mat &tColorImg, const double &timestamp);

    void Reset();

protected:
    Camera_Model    mSensor;

    bool        mbUseViewer;
    bool        mbReseted;

    CameraPtr           mCamera;
    Map                 *mMap;
    Tracking            *mTracker;
    LocalMapping        *mLocalMapper;
    MapDrawer           *mMapDrawer;
    Viewer              *mViewer;

    std::thread*    mtLocalMapper;
    std::thread*    mtViewer;

};
}// namesapce

#endif //DSDTM_SYSTEM_H_H
