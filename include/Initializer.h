//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_INITIALIZER_H
#define DSDTM_INITIALIZER_H

#include "Camera.h"
#include "Feature.h"
#include "Frame.h"
#include "Feature_detection.h"
#include "MapPoint.h"


namespace DSDTM
{
struct Feature;
class Frame;
class Feature_detector;

class Initializer
{
public:
    Initializer(Frame &frame, Camera* camera);
    ~Initializer();

    //! Initalize the RGBD camera
    bool Init_RGBDCam(Frame &frame);

    //! Initalize the Monocular camera
    bool Init_MonocularCam(Frame &lastFrame, Frame &currentFrame);

private:
    //! Delete features relate to status
    void ReduceFeatures(Features &_features, std::vector<uchar> _status);
    void ReduceFeatures(std::vector<cv::Point2f> &_points, std::vector<uchar> _status);


public:
    Frame                   mReferFrame;

protected:

    Camera                  *mCam;
    Feature_detector        *mFeature_detector;

    bool            mbInitSuccess;
};
}

#endif //DSDTM_INITIALIZER_H
