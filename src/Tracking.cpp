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
    mMoving_detecter = new Moving_Detecter();
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
        {
            mInitializer = new Initializer(mCurrentFrame, mCam, mMap);
        }
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
            TrackWithLastFrame();

            if(NeedKeyframe())
                CraeteKeyframe();

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
    std::vector<cv::Point2f> tCur_Pts, tLast_Pts, tPts_tmp, tBad_PtsA, tBad_PtsB, tPts_new;
    mLastFrame.Get_Features(tLast_Pts);

    if(tLast_Pts.size()>0)
    {
        // LKT track features
        LKT_Track(tLast_Pts, tCur_Pts);

        // Remove outlines with fundamental matrix using ransac
        cv::Mat F = Reject_FMat(tLast_Pts, tCur_Pts, tBad_PtsA);

        {
            // Motion outline remove, Method 1      Frame differ
            //mCurrentFrame.mDynamicMask = mMoving_detecter->Mod_FrameDiff(mCurrentFrame.mColorImg, mLastFrame.mColorImg,
                                                                         //tCur_Pts, tLast_Pts);

            // Motion outline remove, Method 2      Rarsac
            //Rarsac_F(tCur_Pts, tLast_Pts, tBad_PtsA);

            // Motion outline remove, Method 3      FastMCD
            //mCurrentFrame.mDynamicMask = mMoving_detecter->Mod_FastMCD(mCurrentFrame.mColorImg, tCur_Pts, tLast_Pts);

        }// moving detection

        std::cout << " --- " << tCur_Pts.size() << std::endl;
        if (tCur_Pts.size() < 20)
        {
            mState = Lost;
            std::cout << "Too few features: " << tCur_Pts.size() << " after rarsac" << std::endl;
            return;
        }
    }

    AddNewFeatures(tCur_Pts, tBad_PtsA);
    mCurrentFrame.Get_Features(tPts_tmp);
    std::vector<cv::Point2f> tvNewFeatures(tPts_tmp.begin()+tCur_Pts.size(), tPts_tmp.end());
    std::vector<cv::Point2f> tvGoodFeatures(tPts_tmp.begin(), tPts_tmp.begin()+tCur_Pts.size());
    mCam->Show_Features(mCurrentFrame.mColorImg, tBad_PtsA, tvGoodFeatures, tvNewFeatures);
}

void Tracking::LKT_Track(std::vector<cv::Point2f> &_last_Pts, std::vector<cv::Point2f> &_cur_Pts)
{
    std::vector<uchar> status;
    std::vector<float> tPyrLK_error;

    cv::calcOpticalFlowPyrLK(mLastFrame.mColorImg, mCurrentFrame.mColorImg,
                             _last_Pts, _cur_Pts, status, tPyrLK_error,
                             cv::Size(21, 21), mPyra_levels);

    for (int i = 0; i < _cur_Pts.size(); ++i)
    {
        if (status[i] && !mCam->IsInImage(_cur_Pts[i]))
        {
            status[i] = 0;
        }
    }

    ReduceFeatures(_cur_Pts, status);
    ReduceFeatures(_last_Pts, status);
    ReduceStatus(mvcStatus, status);
    ReduceStatus(mvsTrack_cnt, status);
}

void Tracking::Rarsac_F(std::vector<cv::Point2f> &_last_Pts, std::vector<cv::Point2f> &_cur_Pts,
                        std::vector<cv::Point2f> &_bad_Pts)
{
    std::vector<uchar> status;
    mCurrentFrame.mvGrid_probability = mLastFrame.mvGrid_probability;
    mRarsac_base = Rarsac_base(&mCurrentFrame, _cur_Pts, _last_Pts);
    status = mRarsac_base.RejectFundamental();

    //ReduceFeatures(mLastFrame.mvFeatures, tvStatus);
    ReduceFeatures(_cur_Pts, status, &_bad_Pts);
    ReduceFeatures(_last_Pts, status);
    ReduceStatus(mvcStatus, status);
    ReduceStatus(mvsTrack_cnt, status);
}

cv::Mat Tracking::Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts,
                           std::vector<cv::Point2f> &_bad_Pts)
{
    std::vector<uchar> status;
    cv::Mat F = cv::findFundamentalMat(_last_Pts, _cur_Pts, cv::FM_RANSAC, 1.0, 0.99, status);

    ReduceFeatures(_cur_Pts, status, &_bad_Pts);
    ReduceFeatures(_last_Pts, status);
    ReduceStatus(mvcStatus, status);
    ReduceStatus(mvsTrack_cnt, status);

    return F;
}

void Tracking::AddNewFeatures(std::vector<cv::Point2f> &tCur_Pts, std::vector<cv::Point2f> &tBadPts)
{
    std::vector<cv::Point2f>tPts_tmp, tBad_Pts;

    mCurrentFrame.SetFeatures(tCur_Pts, mvcStatus, mvsTrack_cnt);
    UpdateID(mCurrentFrame.mvFeatures);
    mCurrentFrame.Set_Mask(mvcStatus, mvsTrack_cnt, tBadPts);

    tCur_Pts.clear();
    mCurrentFrame.Get_Features(tCur_Pts);
    //mFeature_detector->Set_ExistingFeatures(mLastFrame.mvFeatures);
    //mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    mFeature_detector->detect(&mCurrentFrame, 10);
}

void Tracking::TrackWithLastFrame()
{
    mCurrentFrame.UndistortFeatures();

    TicToc tc;
    size_t N = mLastFrame.mvFeatures.size();
    for (int i = 0; i < N; ++i)
    {
        Feature tFeature = mLastFrame.mvFeatures[i];
        float z = mLastFrame.Get_FeatureDetph(tFeature.mUnpx);
        if(z<0 || tFeature.mlId==-1)
            continue;

        auto it = std::find_if(mCurrentFrame.mvFeatures.begin(), mCurrentFrame.mvFeatures.end(),
                               boost::bind(&Feature::mlId, _1) == tFeature.mlId);

        std::vector<std::pair<Eigen::Vector3d, cv::Point2f>> test;
        if(it != mCurrentFrame.mvFeatures.end())
        {
            Eigen::Vector3d tCamPoint = mCam->Pixel2Camera(tFeature.mUnpx, z);
            it->mf = tCamPoint;
            std::cout << it->mUnpx << std::endl;
        }

    }
    std::vector<std::pair<char,char>> test;
    test.push_back(std::make_pair<char, char>(2, 2));


    std::cout<< tc.toc() << std::endl;

    Optimizer::PoseOptimization(mCurrentFrame);
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

    size_t tNum = 0;
    for(auto it = mCurrentFrame.mvFeatures.begin(); it!=mCurrentFrame.mvFeatures.end();it++, tNum++)
    {
        float z = mCurrentFrame.Get_FeatureDetph(mCurrentFrame.mvFeatures[tNum].mUnpx);
        if (z<=0 && mCurrentFrame.Find_Observations(tNum))
            continue;

        Eigen::Vector3d tPose = mCurrentFrame.UnProject(mCurrentFrame.mvFeatures[tNum].mUnpx, z);
        MapPoint *tMPoint = new MapPoint(tPose, tKFrame);

        tMPoint->Add_Observation(tKFrame, tNum);

        mCurrentFrame.Add_MapPoint(tNum, tMPoint);

        tKFrame->Add_MapPoint(tMPoint);
        tKFrame->Add_Observations(tNum, tMPoint);

        mMap->AddMapPoint(tMPoint);
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

void Tracking::ComputeMaxBin(std::vector<int> *histo, const int L, std::vector<int> &tLktSets)
{
    const int s = histo[0].size();
    tLktSets = histo[0];
    for (int i = 1; i < L; ++i)
    {
        if(s<histo[i].size())
            tLktSets = histo[i];
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
    mvsTrack_cnt.clear();
    for (size_t i = 0; i < mLastFrame.mvFeatures.size(); ++i)
    {
        mvcStatus.push_back(mLastFrame.mvFeatures[i].mlId);
        mvsTrack_cnt.push_back(mLastFrame.mvFeatures[i].mTrack_cnt);
    }
}

void Tracking::UpdateID(Features &features)
{
    for (int i = 0; i < features.size(); ++i)
    {
        if(features[i].mlId==-1)
            features[i].mlId = mlNextID++;

        features[i].mTrack_cnt++;
    }
}

} //namespace DSDTM