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

    //! Motion Detection with fastMCD
    cv::Mat Mod_FastMCD(const cv::Mat tImg, std::vector<cv::Point2f> tPointsA,
                          std::vector<cv::Point2f> tPointsB);

    //! Motion Detection with Frame diff
    cv::Mat Mod_FrameDiff(const cv::Mat tImgA, const cv::Mat tImgB, std::vector<cv::Point2f> tPointsA,
                          std::vector<cv::Point2f> tPointsB);


};



}// namespace DSDTM





#endif //DSDTM_MOVING_DETECTION_H
