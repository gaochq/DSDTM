//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"



namespace DSDTM
{

long int Tracking::mlNextID = 0;

Tracking::Tracking(Camera *_cam, Map *_map, LocalMapping *tLocalMapping):
        mCam(_cam), mInitializer(static_cast<Initializer*>(NULL)), mMap(_map),
        mLocalMapping(tLocalMapping) ,mProcessedFrames(0)
{
    mState = Not_Init;

    mPyra_levels = Config::Get<int>("Camera.PyraLevels");
    mDepthScale = Config::Get<float>("Camera.depth_scale");
    mMaxIters = Config::Get<int>("Optimization.MaxIter");
    mdMinRotParallax = Config::Get<double>("KeyFrame.min_rot");
    mdMinTransParallax = Config::Get<double>("KeyFrame.min_trans");
    mMinFeatures = Config::Get<int>("KeyFrame.min_features");

    mFeature_detector = new Feature_detector();
}

Tracking::~Tracking()
{

}

void Tracking::Track_RGBDCam(const cv::Mat &colorImg, const double ctimestamp, const cv::Mat &depthImg)
{
    cv::Mat tDImg = depthImg;
    if(!colorImg.data || !depthImg.data)
    {
        std::cout<< "No images" <<std::endl;

        mState = No_Images;
        return;
    }
    //! Be sure the depth image should be sacled
    tDImg.convertTo(tDImg, CV_32F, 1.0f/mDepthScale);
    mCurrentFrame = Frame(mCam, colorImg, ctimestamp, tDImg);

    if(mState==Not_Init)
    {
        if(!mInitializer)
            mInitializer = new Initializer(mCurrentFrame, mCam, mMap);

        if(mInitializer->Init_RGBDCam(mCurrentFrame))
        {
            mState = OK;
            mCurrentFrame.Reset_Gridproba();
            mInitFrame = Frame(mInitializer->mReferFrame);

            mpReferenceKF = mMap->Get_InitialKFrame();
            mlNextID = mInitFrame.mvFeatures.size();
        }
    }
    else 
    {
        Track();


        if(mState==OK)
        {
            TrackWithReferenceFrame();

            if(NeedKeyframe())


            mProcessedFrames++;
        }

    }

    mLastFrame.ResetFrame();
    mLastFrame = Frame(mCurrentFrame);
    mCurrentFrame.ResetFrame();
    Reset_Status();
}

void Tracking::Track()
{
    std::vector<cv::Point2f> tCur_Pts, tLast_Pts, tPts_tmp, tBad_Pts;
    mLastFrame.Get_Features(tLast_Pts);

    //Show_Features(Last_Pts);
    LKT_Track(tCur_Pts, tLast_Pts);
    //std::cout << "  " << tCur_Pts.size();
    Rarsac_F(tCur_Pts, tLast_Pts, tBad_Pts);
    //Reject_FMat(tCur_Pts, tLast_Pts, tBad_Pts);

    //TODO: show image and deal with features
    std::cout << " --- " << tCur_Pts.size() << std::endl;
    if (tCur_Pts.size() < 20)
    {
        mState = Lost;
        std::cout << "Too few features: " << tCur_Pts.size() << " after rarsac" << std::endl;
        return;
    }

    AddNewFeatures(tCur_Pts);
    mCurrentFrame.Get_Features(tPts_tmp);
    std::vector<cv::Point2f> tvNewFeatures(tPts_tmp.begin()+tCur_Pts.size(), tPts_tmp.end());
    std::vector<cv::Point2f> tvGoodFeatures(tPts_tmp.begin(), tPts_tmp.begin()+tCur_Pts.size());
    mCam->Show_Features(mCurrentFrame.mColorImg, tBad_Pts, tvGoodFeatures, tvNewFeatures);
    //std::cout<< mCurrentFrame.mvFeatures.size() << std::endl;
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
    ReduceStatus(mvcStatus, tvStatus);
}

void Tracking::Rarsac_F(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                        std::vector<cv::Point2f> &_bad_Pts)
{
    std::vector<uchar> tvStatus;
    mCurrentFrame.mvGrid_probability = mLastFrame.mvGrid_probability;
    mRarsac_base = Rarsac_base(&mCurrentFrame, _cur_Pts, _last_Pts);
    tvStatus = mRarsac_base.RejectFundamental();

    //ReduceFeatures(mLastFrame.mvFeatures, tvStatus);
    ReduceFeatures(_cur_Pts, tvStatus, &_bad_Pts);
    ReduceFeatures(_last_Pts, tvStatus);
    ReduceStatus(mvcStatus, tvStatus);
}

void Tracking::Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                           std::vector<cv::Point2f> &_bad_Pts)
{
    std::vector<uchar> status;
    cv::findFundamentalMat(_last_Pts, _cur_Pts, cv::FM_RANSAC, 1.0, 0.99, status);

    ReduceFeatures(_cur_Pts, status, &_bad_Pts);
    ReduceFeatures(_last_Pts, status);
    ReduceStatus(mvcStatus, status);
}

void Tracking::AddNewFeatures(std::vector<cv::Point2f> tCur_Pts)
{
    std::vector<cv::Point2f>tPts_tmp, tBad_Pts;

    mCurrentFrame.SetFeatures(tCur_Pts, mvcStatus);
    UpdateID(mCurrentFrame.mvFeatures);

    mFeature_detector->Set_ExistingFeatures(mLastFrame.mvFeatures);
    mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    mFeature_detector->detect(&mCurrentFrame, 10);
}

void Tracking::TrackWithReferenceFrame()
{
    mCurrentFrame.Set_Pose(mLastFrame.Get_Pose());
    mCurrentFrame.Add_Observations(*mpReferenceKF);

    Optimizer::PoseSolver(mCurrentFrame);
    mCurrentFrame.Set_Pose(mpReferenceKF->Get_Pose()*mCurrentFrame.Get_Pose());
}

bool Tracking::NeedKeyframe()
{
    double tMinDepth, tMeanDepth;
    //mCurrentFrame.Get_SceneDepth(tMinDepth, tMeanDepth);
    if(mCurrentFrame.mvFeatures.size() < 30)
        return true;

    if(mProcessedFrames<10)
        return false;

    Sophus::SE3 tDeltaPose = mpReferenceKF->Get_Pose().inverse()*mCurrentFrame.Get_Pose();
    double tRotNorm = tDeltaPose.so3().log().norm();
    double tTransNorm = tDeltaPose.translation().norm();

    if(tRotNorm < mdMinRotParallax && tTransNorm < mdMinTransParallax)
    {
        return false;
    }

    return true;
}

void Tracking::CraeteKeyframe()
{
    KeyFrame *tKFrame = new KeyFrame(mCurrentFrame);
    mpReferenceKF = tKFrame;

    size_t tFeature_Num;
    for(auto it = mCurrentFrame.mvFeatures.begin(); it!=mCurrentFrame.mvFeatures.end();it++)
    {
        float z = mCurrentFrame.Get_FeatureDetph(*it);
        if (z<=0 && mCurrentFrame.Find_Observations(tFeature_Num))
            continue;

        Eigen::Vector3d tPose = mCam->Pixel2Camera(it->mpx, z);
        MapPoint *tMPoint = new MapPoint(tPose, tKFrame);

        tMPoint->Add_Observation(tKFrame, tFeature_Num);

        mCurrentFrame.Add_MapPoint(tFeature_Num, tMPoint);

        tKFrame->Add_MapPoint(tMPoint);
        tKFrame->Add_Observations(tFeature_Num, tMPoint);

        mMap->AddMapPoint(tMPoint);
        tFeature_Num++;
    }


}

void Tracking::Update_LastFrame()
{

}


void Tracking::ReduceFeatures(std::vector<cv::Point2f> &_Points, const std::vector<uchar> _Status,
                              std::vector<cv::Point2f> *_BadPoints)
{
    if(!_BadPoints)
    {
        int j = 0;
        for (int i = 0; i < int(_Points.size()); i++)
        {
            if (_Status[i])
                _Points[j++] = _Points[i];

            //else
                //mvcStatus[i]
        }
        _Points.resize(j);
    }
    else
    {
        int j = 0;
        for (int i = 0; i < int(_Points.size()); i++)
        {
            if (_Status[i])
                _Points[j++] = _Points[i];
            else
                _BadPoints->push_back(_Points[i]);
        }
        _Points.resize(j);
    }
}

void Tracking::ReduceStatus(std::vector<long int> &tStatus, const std::vector<uchar> _Status)
{
    int j = 0;
    for (int i = 0; i < int(tStatus.size()); i++)
    {
        if (_Status[i])
            tStatus[j++] = tStatus[i];
    }
    tStatus.resize(j);
}


void Tracking::Reset_Status()
{
    mvcStatus.clear();
    for (size_t i = 0; i < mLastFrame.mvFeatures.size(); ++i)
    {
        mvcStatus.push_back(mLastFrame.mvFeatures[i].mlId);
    }
}

void Tracking::UpdateID(Features &features)
{
    for (int i = 0; i < features.size(); ++i)
    {
        if(features[i].mlId==-1)
            features[i].mlId = mlNextID++;
    }
}



} //namespace DSDTM