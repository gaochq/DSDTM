//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_TRACKING_H
#define DSDTM_TRACKING_H

#include "Feature_detection.h"
#include "Initializer.h"
#include "Rarsac_base.h"
#include "Keyframe.h"
#include "Map.h"

namespace DSDTM
{

class Camera;
class Feature_detector;
class Frame;
class Rarsac_base;
class Feature_detector;
class Initializer;
class KeyFrame;

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
    Tracking(Camera *_cam, Map *_map);
    ~Tracking();

    //! Tracking on the rgbd camera
    void Track_RGBDCam(const cv::Mat &colorImg, const double ctimestamp, const cv::Mat &depthImg);

    //! Tracking on the
    void Track_Monocular(const cv::Mat &Image, const double TimeStamp);

private:
    //! The main tracking function
    void Track();

    //! Tracking with lkt algorithm
    void LKT_Track(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts);

    //! Reject outliers with RARSAC algorithm
    void Rarsac_F(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                  std::vector<cv::Point2f> &_bad_Pts);
    void Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                     std::vector<cv::Point2f> &_bad_Pts);

    //! Delete outliers according to lkt and fundamental status
    void ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<bool> _Status,
                        std::vector<cv::Point2f> *_BadPoints = nullptr);
    void ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<uchar> _Status,
                        std::vector<cv::Point2f> *_BadPoints = nullptr);

    //! Solve the pose of cunrrent with last keyframe
    void TrackWithReferenceFrame((Frame &tFRame));

public:
    Camera                  *mCam;               //! Camera
    int                     mPyra_levels;

protected:
    float                   mDepthScale;

    Rarsac_base             mRarsac_base;
    Feature_detector        *mFeature_detector;
    Initializer             *mInitializer;

    Tracking_State          mState;             //! Tracking state
    Frame                   mLastFrame;
    Frame                   mCurrentFrame;
    Frame                   mInitFrame;

    Map                     *mMap;

};

}// namespace DSDTM

#endif //DSDTM_TRACKING_H
