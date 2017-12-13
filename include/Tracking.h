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
#include "Optimizer.h"
#include "LocalMapping.h"
#include "Moving_Detection.h"

namespace DSDTM
{

class Camera;
class Feature_detector;
class Frame;
class Rarsac_base;
class Feature_detector;
class Initializer;
class KeyFrame;
class LocalMapping;
class Moving_Detecter;

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
    Tracking(Camera *_cam, Map *_map, LocalMapping *tLocalMapping= nullptr);
    ~Tracking();

    //! Tracking on the rgbd camera
    void Track_RGBDCam(const cv::Mat &colorImg, const double ctimestamp, const cv::Mat &depthImg);

    //! Tracking on the
    void Track_Monocular(const cv::Mat &Image, const double TimeStamp);

    //! Delete outliers according to lkt and fundamental status
    static void ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<uchar> _Status,
                        std::vector<cv::Point2f> *_BadPoints = nullptr);
    void ReduceStatus(std::vector<long int> &tStatus, const std::vector<uchar> _Status);

    //! Found the most large bin
    void ComputeMaxBin(std::vector<int>* histo, const int L, std::vector<int> &tLktSets);

private:
    //! The main tracking function
    void Track();

    //! Tracking with lkt algorithm
    void LKT_Track(std::vector<cv::Point2f> &_last_Pts, std::vector<cv::Point2f> &_cur_Pts);

    //! Reject outliers with RARSAC algorithm
    void Rarsac_F(std::vector<cv::Point2f> &_last_Pts, std::vector<cv::Point2f> &_cur_Pts,
                  std::vector<cv::Point2f> &_bad_Pts);
    cv::Mat Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                     std::vector<cv::Point2f> &_bad_Pts);

    //! Solve the pose of cunrrent with last keyframe
    void TrackWithLastFrame();

    //! Add new features into the frame refer to the detection Grid
    void AddNewFeatures(std::vector<cv::Point2f> &tCur_Pts, std::vector<cv::Point2f> &tLast_Pts);

    //! Reset mvcStatus
    void Reset_Status();

    //! Check whether tracking need keyframe
    bool NeedKeyframe();

    //! Update the ID of features
    void UpdateID(Features &features);

    //! Update Last frame
    void Update_LastFrame();

    //! Create new Keyframe
    void CraeteKeyframe();


public:
    Camera                  *mCam;               //! Camera
    int                     mPyra_levels;
    static long int       mlNextID;

protected:
    float                   mDepthScale;
    int                     mMaxIters;

    Rarsac_base             mRarsac_base;
    Feature_detector        *mFeature_detector;
    Initializer             *mInitializer;

    Tracking_State          mState;             //! Tracking state
    Frame                   mLastFrame;
    Frame                   mCurrentFrame;
    Frame                   mInitFrame;
    KeyFrame*               mpReferenceKF;

    Map                     *mMap;
    std::vector<long int>   mvcStatus;          //! The features status when tracking between adjacent frames
    std::vector<long int >  mvsTrack_cnt;

    size_t                  mProcessedFrames;       //! the number of processed frames after last keyframe inseration
    double                  mdMinRotParallax;
    double                  mdMinTransParallax;
    int                     mMinFeatures;

    LocalMapping            *mLocalMapping;
    Moving_Detecter         *mMoving_detecter;

};

}// namespace DSDTM

#endif //DSDTM_TRACKING_H
