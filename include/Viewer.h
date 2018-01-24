//
// Created by buyi on 18-1-9.
//

#ifndef DSDTM_VIEWER_H
#define DSDTM_VIEWER_H

#include "Camera.h"
#include "Tracking.h"
#include "System.h"


namespace DSDTM
{

class System;
class Tracking;

class Viewer
{
public:
    Viewer(System *tSystem, Tracking *pTracking, Map *tMap);
    ~Viewer();

    //! Draw MapPoints and KeyFrames
    void DrawMapPoints();
    void DrawKeyframes();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3 &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void Run();

    //! Start or Stop the system
    void RequestStart();
    void RequestPause();
    void RequestStop();

    //! Check whether the stop menu is choosed
    bool IsStopped();

    //! Set the Stopped true
    bool Stop();

    void Release();

private:

    double mT;
    float mImageWidth, mImageHeight;

    float mViewerpointX, mViewerpointY, mViewerpointZ, mViewerpointF;

    System          *mSystem;
    Tracking        *mTracker;

    bool        mbStopped;
    bool        mbRequestStop;
    bool        mbPaused;

    Map     *mMap;

    float mfKeyFrameSize;
    float mfKeyFrameLineWidth;
    float mfGraphLineWidth;
    float mfPointSize;
    float mfCameraSize;
    float mfCameraLineWidth;

    Sophus::SE3     mCameraPose;
    Sophus::SE3     mLastCamPose;

};



}// namespace DSDTM



#endif //DSDTM_VIEWER_H
