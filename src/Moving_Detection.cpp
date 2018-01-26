//
// Created by buyi on 17-12-8.
//
#include "Moving_Detection.h"

namespace DSDTM
{

Moving_Detecter::Moving_Detecter()
{

}

Moving_Detecter::~Moving_Detecter()
{

}

cv::Mat Moving_Detecter::Mod_FastMCD(const cv::Mat tImg, std::vector<cv::Point2f> tPointsA,
                                       std::vector<cv::Point2f> tPointsB)
{
    cv::Mat ImageA, ImageB;

    cv::Mat Motion_Mask(tImg.size(), CV_8UC1, cv::Scalar(0));

    if(tPointsA.size() > 4)
    {
        cv::Mat H = cv::findHomography(tPointsA, tPointsB, CV_RANSAC);
        Motion_Mask = Run(tImg, H.ptr<double>(0));
    }

    return Motion_Mask;

}

cv::Mat Moving_Detecter::Mod_FrameDiff(FramePtr tframeA, FramePtr tframeB, std::vector<cv::Point2f> tPointsA,
                                       std::vector<cv::Point2f> tPointsB)
{
    cv::Mat mMask;
    if(tPointsA.size() < 4)
        return mMask;


    cv::Mat tImgA = tframeA->mColorImg;
    cv::Mat tImgB = tframeA->mColorImg;

    cv::Mat tAffine = cv::findHomography(tPointsB, tPointsA, CV_RANSAC);
    cv::warpPerspective(tImgB, mMask, tAffine, mMask.size());

    mMask = tImgA - mMask;

    cv::threshold(mMask, mMask, 50, 200.0, CV_THRESH_BINARY);

    cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    cv::morphologyEx(mMask, mMask, MORPH_OPEN, element);
    cv::dilate(mMask, mMask, element);

    cv::namedWindow("Depth");
    cv::imshow("Depth", mMask);
    cv::waitKey(1);

    return mMask;
}



}// namespace DSDTM
