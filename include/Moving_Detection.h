//
// Created by buyi on 17-12-2.
//
#include "Camera.h"
#include "MCDWrapper.h"


#ifndef DSDTM_MOVING_DETECTION_H
#define DSDTM_MOVING_DETECTION_H

namespace DSDTM
{

class Moving_Detecter: public MCDWrapper
{
public:
    Moving_Detecter();
    ~Moving_Detecter();


    cv::Mat Mod_FastMCD(const cv::Mat tImg, cv::vector<cv::Point2f> tPointsA,
                          cv::vector<cv::Point2f> tPointsB);

    cv::Mat Mod_FrameDiff(const cv::Mat tImgA, const cv::Mat tImgB, cv::vector<cv::Point2f> tPointsA,
                          cv::vector<cv::Point2f> tPointsB);


};



}// namespace DSDTM





#endif //DSDTM_MOVING_DETECTION_H
