//
// Created by buyi on 18-1-9.
//

#ifndef DSDTM_VIEWER_H
#define DSDTM_VIEWER_H

#include "Camera.h"
#include "Tracking.h"
#include "MapDrawer.h"
#include "System.h"


namespace DSDTM
{

class System;
class Tracking;

class Viewer
{
public:
    Viewer(System *tSystem, Tracking *pTracking, MapDrawer *tMapDrawer);
    ~Viewer();

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

    MapDrawer       *mMapDrawer;
    System          *mSystem;
    Tracking        *mTracker;

    bool        mbStopped;
    bool        mbRequestStop;
    bool        mbPaused;

};



}// namespace DSDTM



#endif //DSDTM_VIEWER_H
