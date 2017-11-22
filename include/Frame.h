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
    void SetFeatures(const std::vector<cv::Point2f> &_features, std::vector<long int> tStatus);

    //! Clear all the members in frame
    void ResetFrame();

    //! Reset the probability of rarsac grid
    void Reset_Gridproba();

    //! Return Features size
    const int Get_FeatureSize(){ return mvFeatures.size(); }

    //! Set and get frame pose
    void Set_Pose(Sophus::SE3 _pose) { mT_c2w = _pose;}
    Sophus::SE3 Get_Pose() { return  mT_c2w;}

    //! Get the feature depth from depth image
    float Get_FeatureDetph(const Feature feature);

    //! Add mappoint and observation into frame
    void Add_MapPoint(size_t tFid, MapPoint *tPoint);

    //! Add Feature-Mappoint observations
    void Add_Observations(const KeyFrame &tKframe);

    //! Return Observations
    std::map<size_t, MapPoint*> Get_Observations() const { return mpObservation;};

    //! Get scene depth
    void Get_SceneDepth(double tMinDepth, double tMeanDepth);

    //! Check whether the feature int observations
    bool Find_Observations(size_t tID);

    //! Unproject pixel into world
    Eigen::Vector3d UnProject(const cv::Point2f tPixel, const float d);

public:
    typedef std::shared_ptr<Frame> FramePtr;

    Features                mvFeatures;
    unsigned long           mlId;
    static unsigned long    mlNextId;

    double                  mdCloTimestamp;         // color image timestamp
    cv::Mat                 mColorImg;
    std::vector<cv::Mat>    mvImg_Pyr;

    std::vector<double>     mvGrid_probability;
    cv::Mat                 mDepthImg;
    Camera*                 mCamera;

    std::vector<MapPoint*>  mvMapPoints;
    std::map<size_t, MapPoint*>  mpObservation;


protected:

    int                     mPyra_levels;

    Sophus::SE3             mT_c2w;

    double                  mMinDepth;
    double                  mMeanDepth;

};
}// namespace DSDTM


#endif //DSDTM_FRAME_H
