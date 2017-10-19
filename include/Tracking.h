//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_TRACKING_H
#define DSDTM_TRACKING_H

#include "Feature_detection.h"

namespace DSDTM
{

class Camera;
class Frame;
class Feature_detector;
class Initializer;

class Tracking
{
private:
    enum Tracking_State
    {
        No_Images   = 0,
        Not_Init    = 1,
        OK          = 2,
        Lost        = 3,
        Recoliaze   = 4
    };

private:
    Tracking();
    ~Tracking();

    //! Tracking on the rgbd camera
    void Track_RGBDCam(cv::Mat colorImg, double ctimestamp, cv::Mat depthImg, double dtimestamp);

    //! Tracking on the
    void Track_Monocular(cv::Mat &Image, double TimeStamp);

    //! The main tracking function
    Tracking_State Track();


private:

//    Camera_Model            mCamModel;          //! Cmaera Model
//    Camera                  mCam;               //! Camera

    Tracking_State          mState;             //! Tracking state
    Frame                   *mLastFrame;
    Frame                   *mCurrentFrame;

    Initializer             *mInitializer;

};

}// namespace DSDTM

#endif //DSDTM_TRACKING_H
