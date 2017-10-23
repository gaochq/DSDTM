//
// Created by buyi on 17-10-19.
//

#include "Initializer.h"

namespace DSDTM
{

    Initializer::Initializer(Frame &frame):
            mReferFrame(frame)
    {

    }

    Initializer::~Initializer()
    {

    }

    bool Initializer::Init_RGBDCam(Frame &frame)
    {
        mFeature_detector = new Feature_detector(frame.mCamera->mheight, frame.mCamera->mwidth);
        mFeature_detector->detect(&frame, 20);
        if(frame.mvFeatures.size()>0)
            return true;
        else
            return false;
    }

    bool Initializer::Init_MonocularCam(Frame &lastFrame, Frame &currentFrame)
    {
        return true;
    }


}// namespace DSDTM