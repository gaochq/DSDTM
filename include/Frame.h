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
typedef std::vector<Feature> Features;

class Frame
{
public:
    Frame();
    Frame(Frame &frame);
    Frame(Camera* _cam, cv::Mat _colorIag, double _timestamp);
    Frame(Camera* _cam, cv::Mat _colorImg, double _ctimestamp, cv::Mat _depthImg, double _dtimestamp);
    ~Frame();

    //! Initalize the Frame
    void InitFrame();

    //! Compute thr image pyramid
    void ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr);

    //! Return keypoints from features
    void Get_Features(std::vector<cv::Point2f> &KeyPoints);

    //! Return keypoints from features
    void SetFeatures(const std::vector<cv::Point2f> &KeyPoints);

    //! Clear all the members in frame
    void ResetFrame();

    //! Reset the probability of rarsac grid
    void Reset_Gridproba();

    //! Return Features size
    const int Get_FeatureSize(){ return mvFeatures.size(); }

    //! Set frame pose
    void Set_Pose(Sophus::SE3 _pose) { mT_c2w = _pose;}

    //! Get the feature depth from depth image
    float Get_FeatureDetph(const Feature feature);

    //! Add mappoint into frame
    void Add_MapPoint(MapPoint *_Point);

public:
    typedef std::shared_ptr<Frame> FramePtr;

    Features               mvFeatures;
    unsigned long           mlId;
    unsigned long           mlNextId;

    double                  mdCloTimestamp;         // color image timestamp
    double                  mdDepTimestamp;         // deoth image timestamp
    cv::Mat                 mColorImg;
    std::vector<cv::Mat>    mvImg_Pyr;

    std::vector<double>     mvGrid_probability;
    cv::Mat                 mDepthImg;
    Camera*                 mCamera;

    std::vector<MapPoint*>  mvMapPoints;

protected:
    Sophus::SE3             mT_c2w;
    int                     mPyra_levels;

};
}// namespace DSDTM


#endif //DSDTM_FRAME_H
