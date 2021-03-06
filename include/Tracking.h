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
#include "Feature_alignment.h"
#include "Sprase_ImageAlign.h"
#include "Viewer.h"

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
class Sprase_ImgAlign;
class MapDrawer;
class Viewer;

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
    Tracking(CameraPtr _cam, Map *_map, LocalMapping *tLocalMapping);
    ~Tracking();

    //! Tracking on the rgbd camera
    Sophus::SE3 Track_RGBDCam(const cv::Mat &colorImg, const cv::Mat &depthImg, const double ctimestamp);

    //! Tracking on the
    void Track_Monocular(const cv::Mat &Image, const double TimeStamp);

    //! Delete outliers according to lkt and fundamental status
    static void ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<uchar> _Status,
                        std::vector<cv::Point2f> *_BadPoints = nullptr);
    void ReduceStatus(std::vector<long int> &tStatus, const std::vector<uchar> _Status);

    //! Found the most large bin
    void ComputeMaxBin(std::vector<int>* histo, const int L, std::vector<int> &tLktSets);

    //! Set the Viewer
    void SetViewer(Viewer *tViewer);

    //! Count the time of different part
    void CostTimeCount();

    //! Reset Tracker
    void Reset();

    //! Get the Tracker state
    int GetState() { return mState;}

private:

    //! Create Keyframe and MapPoints after initialization
    bool CreateInitialMapRGBD();

    //! Solve the pose of cunrrent with last keyframe
    bool TrackWithLastFrame();

    //! Track with lcoal map, and add more mappoint into current frame
    bool TrackWithLocalMap();

    //! Updata local map keyframes, I choose the 10 most closer to current frame
    void UpdateLocalMap();

    //! Get frames have an overlapping field of current view
    void GetCloseKeyFrames(const Frame *tFrame, std::list<std::pair<KeyFrame*, double> >& tClose_kfs, KeyFrame* tClosetKf = static_cast<KeyFrame*>(NULL)) const;

    //! Reset mvcStatus
    void Reset_Status();

    //! Check whether tracking need keyframe
    bool NeedKeyframe();

    //! Update the ID of features
    void UpdateID(Features &features);

    //! Create new Keyframe
    void CraeteKeyframe();

    //! Try to relocalize the camera
    bool Relocalization();

    //! Motion removal using scene flow
    void MotionRemoval();
    void MotionRemovalTest1();
    void MotionRemovalTest2();


public:
    CameraPtr               mCam;               //! Camera
    int                     mMaxPyra_levels;
    int                     mMinPyra_levels;
    static long int         mlNextID;

    std::vector<std::pair<double, Sophus::SE3>> mTrajectory;
protected:
    float                   mDepthScale;
    int                     mMaxIters;

    Rarsac_base             mRarsac_base;
    Feature_detector        *mFeature_detector;
    Initializer             *mInitializer;

    Tracking_State          mState;             //! Tracking state
    FramePtr                mLastFrame;
    FramePtr                mCurrentFrame;
    FramePtr                mInitFrame;
    KeyFrame*               mpLastKF;

    Map                     *mMap;
    std::vector<long int>   mvcStatus;          //! The features status when tracking between adjacent frames
    std::vector<long int>   mvsTrack_cnt;

    size_t                  mProcessedFrames;       //! the number of processed frames after last keyframe inseration
    double                  mdMinRotParallax;
    double                  mdMinTransParallax;
    double                  mdMinDist;
    int                     mMinFeatures;

    LocalMapping            *mLocalMapping;
    Moving_Detecter         *mMoving_detecter;
    Feature_Alignment       *mFeature_Alignment;
    Sprase_ImgAlign         *mSprase_ImgAlign;
    Viewer                  *mViewer;


    std::vector<KeyFrame*>  mvpLocalKeyFrames;
    std::map<MapPoint*, KeyFrame*>  mvpLocalMapPoints;

    std::vector<double> mvImageAlignTime;
    std::vector<double> mvFeatureAligTime;
    std::vector<double> mvMotionDetection;
    std::vector<double> mvKeyFrameJudgeTime;
    std::vector<double> mvLocalBATime;
    std::vector<double> mvKeyframeCreation;

    std::vector<int> mvFeatureNumCounter;
};

}// namespace DSDTM

#endif //DSDTM_TRACKING_H
