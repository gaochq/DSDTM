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
class Camera;
class KeyFrame;

class Frame
{
public:
    Frame();
    Frame(Frame &frame);
    Frame(Camera* _cam, cv::Mat _colorIag, double _timestamp);
    Frame(Camera* _cam, cv::Mat _colorImg, double _ctimestamp, cv::Mat _depthImg);
    ~Frame();

    //! Initalize the Frame
    void InitFrame();

    //! Compute thr image pyramid
    void ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr);

    //! Return keypoints from features
    void Get_Features(std::vector<cv::Point2f> &KeyPoints);

    //! Return keypoints from features
    void SetFeatures(const std::vector<cv::Point2f> &_features);
    void SetFeatures(const std::vector<cv::Point2f> &_features, std::vector<long int> tStatus,
                     std::vector<long int> tTrack_cnt);

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
    float Get_FeatureDetph(const Feature feature);
    float Get_FeatureDetph(const cv::Point2f feature);

    //! Add mappoint and observation into frame
    void Add_MapPoint(size_t tFid, MapPoint *tPoint);

    //! Add Feature-Mappoint observations
    void Add_Observations(const KeyFrame &tKframe);

    //! Add Feature
    void Add_Feature(Feature tfeature);

    //! Return Observations
    std::map<size_t, MapPoint*> Get_Observations() const { return mpObservation;};

    //! Get scene depth
    void Get_SceneDepth(double tMinDepth, double tMeanDepth);

    //! Check whether the feature int observations
    bool Find_Observations(size_t tID);

    //! Undistort Features
    void UndistortFeatures();

    //! Unproject pixel into world
    Eigen::Vector3d UnProject(const cv::Point2f tPixel, const float d);

    //! Set the image mask for feature extraction and filter
    void Set_Mask(std::vector<long int> &tlId, std::vector<long int> &tTrackCnt,
                  std::vector<cv::Point2f> tBadPts);

    //! Check if this point is visible in current frame
    bool isVisible(const Eigen::Vector3d tPose, int tBoundary = 0) const;

    //! Reset the probability of rarsac grid
    void Reset_Gridproba();

    //! World to camera
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d &Point);


public:
    Features                    mvFeatures;
    std::vector<cv::Point2f>    mvFeaturesUn;           //Features undistorted

    unsigned long           mlId;
    static unsigned long    mlNextId;

    double                  mdCloTimestamp;         // color image timestamp
    cv::Mat                 mColorImg;
    std::vector<cv::Mat>    mvImg_Pyr;

    std::vector<double>     mvGrid_probability;
    cv::Mat                 mDepthImg;
    Camera*                 mCamera;

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
