//
// Created by buyi on 17-11-22.
//

#ifndef DSDTM_SYSTEM_H_H
#define DSDTM_SYSTEM_H_H

#include "Camera.h"
#include "Tracking.h"
#include "Map.h"
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

    Sophus::SE3 TrackRGBD(const cv::Mat &tColorImg, const cv::Mat &tDepthImg, const double &timestamp);

    Sophus::SE3 TrackMonocular(cv::Mat &tColorImg, const double &timestamp);

    //! Pause the System
    void RequestPause();
    void RequestStart();

    //! Save camera trajectory
    void SaveKeyFrameTrajectoryTUM(const string &filename);
    void SaveCameraTrajectory(const string &filename);

    void Reset();

    int ReturnKeyFramesSize() { return mMap->GetAllKeyFrames().size(); }
    int ReturnMapPointsSize() { return mMap->GetAllMapPoints().size(); }

    void OutputTimeCounter();

    int GetSystemState();

protected:
    Camera_Model    mSensor;

    bool        mbUseViewer;
    bool        mbReseted;
    bool        mbPaused;

    CameraPtr           mCamera;
    Map                 *mMap;
    Tracking            *mTracker;
    LocalMapping        *mLocalMapper;
    Viewer              *mViewer;

    std::thread*    mtLocalMapper;
    std::thread*    mtViewer;

};
}// namesapce

#endif //DSDTM_SYSTEM_H_H
