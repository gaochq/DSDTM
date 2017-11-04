//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"

namespace DSDTM
{
Tracking::Tracking(Camera *_cam):
        mCam(_cam)
{
    mState = Not_Init;
    mPyra_levels = Config::Get<int>("Camera.PyraLevels");
    mFeature_detector = new Feature_detector();
}

Tracking::~Tracking()
{

}

void Tracking::ReduceFeatures(std::vector<cv::Point2f> &_Points, std::vector<uchar> _Status)
{
    for (int i = 0; i < _Points.size(); ++i)
    {
        if(!_Status[i])
            _Points.erase(_Points.begin() + i);
    }
}

void Tracking::ReduceFeatures(std::vector<cv::Point2f> &_Points, std::vector<bool> _Status)
{
    for (int i = 0; i < _Points.size(); ++i)
    {
        if(!_Status[i])
            _Points.erase(_Points.begin() + i);
    }
}

void Tracking::Track_RGBDCam(cv::Mat colorImg, double ctimestamp, cv::Mat depthImg, double dtimestamp)
{
    if(!colorImg.data || !depthImg.data)
    {
        std::cout<< "No images" <<std::endl;

        mState = No_Images;
        return;
    }
    mCurrentFrame = Frame(mCam, colorImg, ctimestamp, depthImg, dtimestamp);

    if(mState==Not_Init)
    {
        if(!mInitializer)
            mInitializer = new Initializer(mCurrentFrame, mCam);

        if(mInitializer->Init_RGBDCam(mCurrentFrame))
        {
            mState = OK;
            mCurrentFrame.Reset_Gridproba();
            mInitFrame = Frame(mCurrentFrame);
        }
    }
    else
        Track();

    mLastFrame.ResetFrame();
    mLastFrame = Frame(mCurrentFrame);
    mCurrentFrame.ResetFrame();
}

void Tracking::Track()
{
    std::vector<cv::Point2f> Cur_Pts, Last_Pts, Pts_tmp;
    mLastFrame.Get_Features(Last_Pts);

    LKT_Track(Cur_Pts, Last_Pts);
    if(Cur_Pts.size()!=Last_Pts.size())
        std::cerr <<"false"<<std::endl;
    Show_Features(Cur_Pts);
    Rarsac_F(Cur_Pts, Last_Pts);
    //Reject_FMat(Cur_Pts, Last_Pts);

    //TODO: show image and deal with features
    //std::cout<<"  " <<Cur_Pts.size() <<std::endl;
    Show_Features(Cur_Pts);

    mCurrentFrame.SetFeatures(Cur_Pts);
    mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    if(Cur_Pts.size() < 200)
        mFeature_detector->detect(&mCurrentFrame, 20);
    //std::cout<< mCurrentFrame.mvFeatures.size() << std::endl;
    mCurrentFrame.Get_Features(Pts_tmp);

}

void Tracking::LKT_Track(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts)
{
    std::vector<uchar> tvStatus;
    std::vector<float> tPyrLK_error;

    cv::calcOpticalFlowPyrLK(mLastFrame.mColorImg, mCurrentFrame.mColorImg,
                             _last_Pts, _cur_Pts, tvStatus, tPyrLK_error,
                             cv::Size(21, 21), mPyra_levels);

    for (int i = 0; i < _cur_Pts.size(); ++i)
    {
        if (mCam->IsInImage(_cur_Pts[i]))
            continue;
        else
        {
            _cur_Pts.erase(_cur_Pts.begin() + i);
            _last_Pts.erase(_last_Pts.begin() + i);
            i--;
        }

    }

    ReduceFeatures(_cur_Pts, tvStatus);
    ReduceFeatures(_last_Pts, tvStatus);
//        ReduceFeatures(mLastFrame.mvFeatures, tvStatus);
}

void Tracking::Rarsac_F(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts)
{
    std::vector<bool> tvStatus;
    mCurrentFrame.mvGrid_probability = mLastFrame.mvGrid_probability;
    mRarsac_base = Rarsac_base(&mCurrentFrame, _cur_Pts, _last_Pts);
    tvStatus = mRarsac_base.RejectFundamental();

    //ReduceFeatures(mLastFrame.mvFeatures, tvStatus);
    ReduceFeatures(_cur_Pts, tvStatus);
    ReduceFeatures(_last_Pts, tvStatus);

}

void Tracking::Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts)
{
    std::vector<uchar> status;
    cv::findFundamentalMat(_cur_Pts, _last_Pts, cv::FM_RANSAC, 3, 0.99, status);

    ReduceFeatures(_cur_Pts, status);
    ReduceFeatures(_cur_Pts, status);
}

void Tracking::Show_Features(std::vector<cv::Point2f> _features)
{
    cv::Mat Image_new = mCurrentFrame.mColorImg.clone();
    mCam->Draw_Features(Image_new, _features);

    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", Image_new);
    cv::waitKey(1);
}



}