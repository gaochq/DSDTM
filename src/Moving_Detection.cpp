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

    cv::Mat H = cv::findHomography(tPointsA, tPointsB, CV_RANSAC);

    //cv::pyrDown(tImg, ImageB);
    cv::Mat Motion_Mask = Run(tImg, H.ptr<double>(0));

    return Motion_Mask;

}

cv::Mat Moving_Detecter::Mod_FrameDiff(FramePtr tframeA, FramePtr tframeB, std::vector<cv::Point2f> tPointsA,
                                       std::vector<cv::Point2f> tPointsB)
{

    cv::Mat tImgCA = tframeA->mColorImg;
    cv::Mat tImgCB = tframeB->mColorImg;
    cv::Mat tImgDA = tframeA->mDepthImg;
    cv::Mat tImgDB = tframeB->mDepthImg;

    cv::Mat mMask, mMask1;
    cv::Mat tAffine = cv::findHomography(tPointsB, tPointsA, CV_RANSAC);

    cv::warpPerspective(tImgCB, mMask, tAffine, mMask.size());
    cv::warpPerspective(tImgDB, mMask1, tAffine, mMask.size());

    mMask = tImgCA - mMask;
    mMask1 = tImgDA - mMask1;

    cv::imwrite("depth.png", mMask);

    //cv::threshold(mMask, mMask, 150, 255.0, CV_THRESH_TOZERO_INV);
    cv::threshold(mMask, mMask, 50, 255.0, CV_THRESH_TOZERO);


    cv::Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    cv::morphologyEx(mMask, mMask, MORPH_OPEN, element);
    cv::dilate(mMask, mMask, element);

    cv::namedWindow("Depth");
    cv::imshow("Depth", mMask);
    cv::waitKey(1);


    return mMask;
}



}// namespace DSDTM
