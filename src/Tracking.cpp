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
    }

    Tracking::~Tracking()
    {

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
        mCurrentFrame = Frame(mCam,colorImg, ctimestamp, depthImg, dtimestamp);

        if(mState==Not_Init)
        {
            mInitializer = new Initializer(mCurrentFrame);

            if(mInitializer->Init_RGBDCam(mCurrentFrame))
            {
                mState = OK;
                mLastFrame.Reset_Gridproba();
                mInitFrame = Frame(mCurrentFrame);

                return;
            }
        }
        else
            Track();

        mLastFrame = Frame(mCurrentFrame);
    }

    void Tracking::Track()
    {
        std::vector<cv::Point2f> Cur_Pts, Last_Pts;
        mLastFrame.GetKeypoints(Last_Pts);

        LKT_Track(Cur_Pts, Last_Pts);




    }

    void Tracking::LKT_Track(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts)
    {
        std::vector<uchar> tvStatus;
        std::vector<float> tPyrLK_error;

        cv::calcOpticalFlowPyrLK(mLastFrame.mColorImg, mCurrentFrame.mColorImg,
                                 _last_Pts, _cur_Pts, tvStatus, tPyrLK_error,
                                 cv::Size(21, 21), mPyra_levels);

        std::vector<cv::Point2f>::iterator Last_Pts_it = _last_Pts.begin();
        std::vector<cv::Point2f>::iterator Cur_Pts_it = _cur_Pts.begin();
//        std::vector<Feature>::iterator Last_Features_it = mLastFrame.mvFeatures.begin();

        for (int i = 0; i < _last_Pts.size(); ++i)
        {
            if(!tvStatus[i])
            {
                _last_Pts.erase(Last_Pts_it);
                _cur_Pts.erase(Cur_Pts_it);
//                mLastFrame.mvFeatures.erase(Last_Features_it);

                continue;
            }
            Last_Pts_it++;
            Cur_Pts_it++;
//            Last_Features_it++;
        }


    }

    void Tracking::Rarsac_F(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts)
    {
        std::vector<bool> tvStatus;
        mRarsac_base = Rarsac_base(&mCurrentFrame, _cur_Pts, _last_Pts);
        tvStatus = mRarsac_base.RejectFundamental();


    }


}