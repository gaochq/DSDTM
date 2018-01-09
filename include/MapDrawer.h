//
// Created by buyi on 18-1-9.
//

#ifndef DSDTM_MAPDRAWER_H
#define DSDTM_MAPDRAWER_H


#include "Camera.h"
#include "Tracking.h"


namespace DSDTM
{

class MapDrawer
{
public:
    MapDrawer(Map *pMap);

    void DrawMapPoints();
    void DrawKeyframes();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3 &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);


protected:

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

#endif //DSDTM_MAPDRAWER_H
