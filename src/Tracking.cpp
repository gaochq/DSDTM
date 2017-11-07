//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"

namespace DSDTM
{
Tracking::Tracking(Camera *_cam):
        mCam(_cam), mInitializer(static_cast<Initializer*>(NULL))
{
    mState = Not_Init;
    mPyra_levels = Config::Get<int>("Camera.PyraLevels");
    mDepthScale = Config::Get<float>("Camera.depth_scale");

    mFeature_detector = new Feature_detector();
}

Tracking::~Tracking()
{

}

void Tracking::ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<uchar> _Status)
{
    int j = 0;
    for (int i = 0; i < int(_Points.size()); i++)
        if (_Status[i])
            _Points[j++] = _Points[i];
    _Points.resize(j);
}

void Tracking::ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<bool> _Status)
{
    int j = 0;
    for (int i = 0; i < int(_Points.size()); i++)
        if (_Status[i])
            _Points[j++] = _Points[i];
    _Points.resize(j);
}

void Tracking::Track_RGBDCam(const cv::Mat &colorImg, const double ctimestamp, const cv::Mat &depthImg)
{
    cv::Mat tDImg;
    if(!colorImg.data || !depthImg.data)
    {
        std::cout<< "No images" <<std::endl;

        mState = No_Images;
        return;
    }

    //! Be sure the depth image should be sacled
    depthImg.convertTo(tDImg, CV_32F, 1.0f/mDepthScale);

    mCurrentFrame = Frame(mCam, colorImg, ctimestamp, tDImg);

    /*
    if(mState==Not_Init)
    {
        if(!mInitializer)
            mInitializer = new Initializer(mCurrentFrame, mCam);

        if(mInitializer->Init_RGBDCam(mCurrentFrame))
        {
            mState = OK;
            mCurrentFrame.Reset_Gridproba();
            mInitFrame = Frame(mInitializer->mReferFrame);
        }
    }
    else
     */
        Track();

    mLastFrame.ResetFrame();
    mLastFrame = Frame(mCurrentFrame);
    mCurrentFrame.ResetFrame();
}

void Tracking::Track()
{
    std::vector<cv::Point2f> Cur_Pts, Last_Pts, Pts_tmp;
    mLastFrame.Get_Features(Last_Pts);

    if(Last_Pts.size()>0)
    {
        //Show_Features(Last_Pts);
        LKT_Track(Cur_Pts, Last_Pts);
        if (Cur_Pts.size() < 20) {
            std::cout << "Too few features: " << Cur_Pts.size() << " after KLT" << std::endl;
            //return;
        }
        std::cout << "  " << Cur_Pts.size();
        //Rarsac_F(Cur_Pts, Last_Pts);
        Reject_FMat(Cur_Pts, Last_Pts);

        //TODO: show image and deal with features
        std::cout << " --- " << Cur_Pts.size() << std::endl;
        if (Cur_Pts.size() < 20) {
            std::cout << "Too few features: " << Cur_Pts.size() << " after rarsac" << std::endl;
            //return;
        }
    }
    Show_Features(Cur_Pts, 0);

    mCurrentFrame.SetFeatures(Cur_Pts);
    mFeature_detector->Set_ExistingFeatures(mLastFrame.mvFeatures);
    mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    mFeature_detector->detect(&mCurrentFrame, 20);
    mCurrentFrame.Get_Features(Pts_tmp);
    std::vector<cv::Point2f> fea1(Pts_tmp.begin()+Cur_Pts.size(), Pts_tmp.end());
    std::vector<cv::Point2f> fea2(Pts_tmp.begin(), Pts_tmp.begin()+Cur_Pts.size());
    Show_Features(fea2, fea1);
    std::cout<< mCurrentFrame.mvFeatures.size() << std::endl;
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
        if (tvStatus[i] && !mCam->IsInImage(_cur_Pts[i]))
        {
            tvStatus[i] = 0;
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
    cv::findFundamentalMat(_last_Pts, _cur_Pts, cv::FM_RANSAC, 1.0, 0.99, status);

    ReduceFeatures(_cur_Pts, status);
    ReduceFeatures(_last_Pts, status);
}

void Tracking::Show_Features(const std::vector<cv::Point2f> _features, int _color)
{
    cv::Mat Image_new = mCurrentFrame.mColorImg.clone();
    mCam->Draw_Features(Image_new, _features, _color);

    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", Image_new);
    cv::waitKey(1);
}
void Tracking::Show_Features(const std::vector<cv::Point2f> _features1, const std::vector<cv::Point2f> _features2)
{
    cv::Mat Image_new = mCurrentFrame.mColorImg.clone();
    mCam->Draw_Features(Image_new, _features1, 0);
    mCam->Draw_Features(Image_new, _features2, 1);
    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", Image_new);
    cv::waitKey(1);
}

}