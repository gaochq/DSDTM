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
    Moving_Detecter()
    {}
    ~Moving_Detecter()
    {}

    cv::Mat Mod_Detection(cv::Mat tImg, cv::vector<cv::Point2f> tPointsA, cv::vector<cv::Point2f> tPointsB)
    {
        cv::Mat tMask(tImg.size(), CV_8UC1, cv::Scalar::all(0));
        for (int j = 0; j < tPointsA.size(); ++j)
        {
            tMask.at<uchar>(tPointsA[j].y, tPointsA[j].x) = 255;
        }
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::dilate(tMask, tMask, element);

        cv::Mat H = cv::findHomography(tPointsA, tPointsB, CV_RANSAC);
        Run(tImg, H.ptr<double>(0), tMask);
    }


};



}// namespace DSDTM





#endif //DSDTM_MOVING_DETECTION_H
