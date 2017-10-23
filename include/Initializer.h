//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_INITIALIZER_H
#define DSDTM_INITIALIZER_H

#include "Camera.h"
#include "Frame.h"
#include "Feature_detection.h"


namespace DSDTM
{

class Frame;
class Feature_detector;

class Initializer
{
public:
    Initializer(Frame &frame);
    ~Initializer();

    //! Initalize the RGBD camera
    bool Init_RGBDCam(Frame &frame);

    //! Initalize the Monocular camera
    bool Init_MonocularCam(Frame &lastFrame, Frame &currentFrame);

private:


public:
    Frame           mReferFrame;
    Feature_detector        *mFeature_detector;
};
}

#endif //DSDTM_INITIALIZER_H
