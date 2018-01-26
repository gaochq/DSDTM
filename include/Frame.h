//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_FRAME_H
#define DSDTM_FRAME_H


#include "Camera.h"
#include "Feature.h"
#include "MapPoint.h"

namespace DSDTM
{
class MapPoint;
class KeyFrame;

class Frame
{

public:
    Frame();
    Frame(Frame &frame);
    Frame(CameraPtr _cam, cv::Mat _colorIag, double _timestamp);
    Frame(CameraPtr _cam, cv::Mat _colorImg, cv::Mat _depthImg, double _ctimestamp);
    ~Frame();

    //! Initalize the Frame
    void InitFrame();

    //! Compute thr image pyramid
    void ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr);

    //! Add Feature
    void Add_Feature(Feature *tfeature, bool tbNormal = true);

    //! Undistort Features
    void UndistortFeatures();

    //! Return keypoints from features
    void Get_Features(std::vector<cv::Point2f> &KeyPoints);

    //! Clear all the members in frame
    void ResetFrame();

    //! Return Features size
    const int Get_FeatureSize(){ return mvFeatures.size(); }

    //! Set and get frame pose
    void Set_Pose(Sophus::SE3 _pose);
    Sophus::SE3 Get_Pose() const { return  mT_c2w; }

    //! Get Camera center in world coordinate system
    Eigen::Vector3d Get_CameraCnt() const { return mOw; }

    //! Get the feature depth from depth image
    float Get_FeatureDetph(const Feature* feature);
    float Get_FeatureDetph(const cv::Point2f feature);

    //! Add mappoint and observation into frame
    void Add_MapPoint(MapPoint *tMPoint, int tIdx);
    void Add_MapPoint(MapPoint *tMPoint);

    //! Add Feature-Mappoint observations
    void Add_Observations(const KeyFrame &tKframe);

    //! Return Observations
    std::map<size_t, MapPoint*> Get_Observations() const { return mpObservation;};

    //! Get scene depth
    bool Get_SceneDepth(double &tMinDepth, double &tMeanDepth);

    //! Check whether the feature int observations
    bool Find_Observations(size_t tID);

    //! Unproject pixel into world
    Eigen::Vector3d UnProject(const cv::Point2f tPixel, const float d);

    //! Set the image mask for feature extraction and filter
    void Set_Mask();

    //! Check if this point is visible in current frame
    bool isVisible(const Eigen::Vector3d tPose, int tBoundary = 0) const;

    //! Reset the probability of rarsac grid
    void Reset_Gridproba();

    //! World to camera
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d &Point);

    //! Get common MapPoints and Features
    void Get_CorrespondMp();

    //! Get vaild MapPoints number
    int Get_VaildMpNums();

    //! Remove the moving Features
    void Motion_Removal(const cv::Mat tMask);


public:
    Features                mvFeatures;

    unsigned long           mlId;
    static unsigned long    mlNextId;

    double                  mdCloTimestamp;         // color image timestamp
    cv::Mat                 mColorImg;
    std::vector<cv::Mat>    mvImg_Pyr;

    std::vector<double>     mvGrid_probability;
    cv::Mat                 mDepthImg;
    CameraPtr               mCamera;

    cv::Mat                 mImgMask;
    cv::Mat                 mDynamicMask;

    std::vector<MapPoint*>                      mvMapPoints;
    std::map<size_t, MapPoint*>                 mpObservation;


protected:

    int                     mPyra_levels;
    int                     mMin_Dist;

    Sophus::SE3             mT_c2w;                 // pose from world to camera
    Sophus::SO3             mR_w2c;
    Eigen::Vector3d         mOw;

    double                  mMinDepth;
    double                  mMeanDepth;

};

typedef std::shared_ptr<Frame> FramePtr;
}// namespace DSDTM


#endif //DSDTM_FRAME_H
