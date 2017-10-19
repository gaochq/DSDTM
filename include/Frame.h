//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_FRAME_H
#define DSDTM_FRAME_H

#include "Camera.h"
#include "Feature.h"

namespace DSDTM
{

class Camera;
typedef std::vector<Feature> mFeatures;

class Frame
{
public:
    Frame(Frame &frame);
    Frame(Camera* _cam, cv::Mat _colorIag, double _timestamp);
    Frame(Camera* _cam, cv::Mat _colorImg, double _ctimestamp, cv::Mat _depthImg, double _dtimestamp);
    ~Frame();

    //! Initalize the Frame
    void InitFrame();

    //! Compute thr image pyramid
    void ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr);

public:
    typedef std::shared_ptr<Frame> FramePtr;
    Camera*                 mCamera;
    unsigned long           mId;
    double                  mdCloTimestamp;         // color image timestamp
    double                  mdDepTimestamp;         // deoth image timestamp
    Sophus::SE3             mT_c2w;
    cv::Mat                 mColorImg;
    std::vector<cv::Mat>    mvImg_Pyr;
    cv::Mat                 mDepthImg;
    mFeatures               mvFeatures;
    int                     mPyra_levels;

};
}// namespace DSDTM


#endif //DSDTM_FRAME_H
