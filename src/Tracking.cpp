//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"

namespace DSDTM
{
    Tracking::Tracking()
    {
        mState = Not_Init;
    }

    Tracking::~Tracking()
    {

    }

    void Tracking::Track_RGBDCam(cv::Mat colorImg, double ctimestamp, cv::Mat depthImg, double dtimestamp)
    {
        if(!colorImg.data || depthImg.data)
        {
            std::cout<< "No images" <<std::endl;

            mState = No_Images;
            return;
        }
//        mCurrentFrame = new Frame(colorImg, ctimestamp, depthImg, dtimestamp);

        if(mState==Not_Init)
        {
        }

    }
}