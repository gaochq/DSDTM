//
// Created by buyi on 17-10-19.
//

#ifndef DSDTM_INITIALIZER_H
#define DSDTM_INITIALIZER_H

#include "Camera.h"


namespace DSDTM
{

class Frame;
class Feature_detector;

class Initializer
{
public:
    Initializer();
    ~Initializer();

private:
    bool Init_RGBDCam(Frame *frame);
    bool Init_MonocularCam(Frame *lastFrame, Frame *currentFrame);


};
}

#endif //DSDTM_INITIALIZER_H
