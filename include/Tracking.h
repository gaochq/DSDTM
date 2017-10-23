//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_TRACKING_H
#define DSDTM_TRACKING_H

#include "Feature_detection.h"
#include "Initializer.h"
#include "Rarsac_base.h"

namespace DSDTM
{

class Camera;
class Feature_detector;
class Frame;
class Rarsac_base;
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

public:
    Tracking(Camera *_cam);
    ~Tracking();

    //! Tracking on the rgbd camera
    void Track_RGBDCam(cv::Mat colorImg, double ctimestamp, cv::Mat depthImg, double dtimestamp);

    //! Tracking on the
    void Track_Monocular(cv::Mat &Image, double TimeStamp);

private:
    //! The main tracking function
    void Track();

    //! Tracking with lkt algorithm
    void LKT_Track(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts);

    //! Reject outliers with RARSAC algorithm
    void Rarsac_F(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts);

    //! Delete outliers according to lkt and fundamental status
    void ReduceFeatures(std::vector<cv::Point2f> &_Points, std::vector<uchar> _Status);
    void ReduceFeatures(std::vector<cv::Point2f> &_Points, std::vector<bool> _Status);

    //! Display image and features
    void Show_Features(std::vector<cv::Point2f> _features);


public:
    Camera                  *mCam;               //! Camera

    Tracking_State          mState;             //! Tracking state
    Frame                   mLastFrame;
    Frame                   mCurrentFrame;
    Frame                   mInitFrame;

    Initializer             *mInitializer;
    int                     mPyra_levels;

    Rarsac_base             mRarsac_base;
    Feature_detector        *mFeature_detector;

};

}// namespace DSDTM

#endif //DSDTM_TRACKING_H
