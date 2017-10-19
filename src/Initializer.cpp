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
        return true;
    }

    bool Initializer::Init_MonocularCam(Frame &lastFrame, Frame &currentFrame)
    {
        return true;
    }


}// namespace DSDTM