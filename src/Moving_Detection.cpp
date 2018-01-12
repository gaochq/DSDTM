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
    cv::Mat tModIMg;
    tImg.copyTo(tModIMg);

    cv::Mat tMask(tImg.size(), CV_8UC1, cv::Scalar::all(0));
    for (int j = 0; j < tPointsA.size(); ++j)
    {
        tMask.at<uchar>(tPointsA[j].y, tPointsA[j].x) = 255;
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(tMask, tMask, element);

    cv::Mat H = cv::findHomography(tPointsA, tPointsB, CV_RANSAC);
    cv::Mat Motion_Mask = Run(tModIMg, H.ptr<double>(0), tMask);

    return Motion_Mask;

}

cv::Mat Moving_Detecter::Mod_FrameDiff(const cv::Mat tImgA, const cv::Mat tImgB, std::vector<cv::Point2f> tPointsA,
                                       std::vector<cv::Point2f> tPointsB)
{
    cv::Mat mMask;
    cv::Mat tAffine = cv::findHomography(tPointsB, tPointsA, CV_RANSAC);
    cv::warpPerspective(tImgB, mMask, tAffine, mMask.size());

    mMask = tImgA - mMask;

    cv::threshold(mMask, mMask, 30, 200.0, CV_THRESH_BINARY);

    cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    cv::morphologyEx(mMask, mMask, MORPH_OPEN, element);
    //cv::dilate(mMask, mMask, element);


    cv::namedWindow("Depth");
    cv::imshow("Depth", mMask);
    cv::waitKey(1);

    return mMask;
}



}// namespace DSDTM
