//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"



namespace DSDTM
{

long int Tracking::mlNextID = 0;

Tracking::Tracking(CameraPtr _cam, Map *_map, LocalMapping *tLocalMapping):
        mCam(_cam), mInitializer(static_cast<Initializer*>(NULL)), mMap(_map),
        mLocalMapping(tLocalMapping) ,mProcessedFrames(0)
{
    mState = Not_Init;

    mMaxPyra_levels = Config::Get<int>("Camera.MaxPyraLevels");
    mMinPyra_levels = Config::Get<int>("Camera.MinPyraLevels");

    mDepthScale = Config::Get<float>("Camera.depth_scale");
    mMaxIters = Config::Get<int>("Optimization.MaxIter");
    mdMinRotParallax = Config::Get<double>("KeyFrame.min_rot");
    mdMinTransParallax = Config::Get<double>("KeyFrame.min_trans");
    mMinFeatures = Config::Get<int>("KeyFrame.min_features");

    mFeature_detector = new Feature_detector();

    mMoving_detecter = new Moving_Detecter();

    mFeature_Alignment = new Feature_Alignment(mCam);

    mSprase_ImgAlign = new Sprase_ImgAlign(mMaxPyra_levels, mMinPyra_levels, mMaxIters);
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
    mCurrentFrame.reset(new Frame(mCam, colorImg, tDImg, ctimestamp));

    if(mState==Not_Init)
    {
        if(!mInitializer)
        {
            mInitializer = new Initializer(mCurrentFrame, mCam, mMap);
        }
        if(mInitializer->Init_RGBDCam(mCurrentFrame))
        {
            mInitFrame = mCurrentFrame;

            if(!CreateInitialMapRGBD())
                return;

            mState = OK;
            mCurrentFrame->Reset_Gridproba();


            mlNextID = mInitFrame->mvFeatures.size();
        }
    }
    else 
    {
        if(mState==OK)
        {
            bool bOK;

            TicToc tc;
            bOK = TrackWithLastFrame();
            std::cout << tc.toc() <<"----";

            if(bOK)
            {
                TrackWithLocalMap();
                std::cout << tc.toc() << std::endl;
                //if (NeedKeyframe())
                //    CraeteKeyframe();
            }
            else
                DLOG(ERROR)<< "Tracking lost with last frame" << std::endl;

            mProcessedFrames++;
        }

    }

    mLastFrame = mCurrentFrame;
    Reset_Status();
}

void Tracking::AddNewFeatures(std::vector<cv::Point2f> &tCur_Pts, std::vector<cv::Point2f> &tBadPts)
{
    std::vector<cv::Point2f>tPts_tmp, tBad_Pts;

    mCurrentFrame->SetFeatures(tCur_Pts, mvcStatus, mvsTrack_cnt);
    UpdateID(mCurrentFrame->mvFeatures);
    mCurrentFrame->Set_Mask(mvcStatus, mvsTrack_cnt, tBadPts);

    tCur_Pts.clear();
    mCurrentFrame->Get_Features(tCur_Pts);
    //mFeature_detector->Set_ExistingFeatures(mLastFrame.mvFeatures);
    //mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    mFeature_detector->detect(mCurrentFrame.get(), 10);
}

bool Tracking::CreateInitialMapRGBD()
{
    mInitFrame->UndistortFeatures();
    KeyFrame *tKFrame = new KeyFrame(mInitFrame.get());

    //TODO detect orb Feature and compute the BOW vector

    mMap->AddKeyFrame(tKFrame);

    for (int i = 0; i < mInitFrame->mvFeatures.size(); ++i)
    {
        float z = mInitFrame->Get_FeatureDetph(mInitFrame->mvFeatures[i]->mpx);

        if(z < 0)
            continue;

        //Eigen::Vector3d tPose = mInitFrame->UnProject(mInitFrame->mvFeatures[i]->mpx, z);
        Eigen::Vector3d tPose = mInitFrame->mvFeatures[i]->mNormal*z;
        tPose = mInitFrame->Get_Pose().inverse()*tPose;
        Eigen::Vector2d tPt = mInitFrame->World2Pixel(tPose);

        Eigen::Vector3d tPose1 = Sophus::SE3(Eigen::Quaterniond(0.0, 0.8227, 0.2148, 0.0), Eigen::Vector3d(0.2263, 0.2262, 2.000)).inverse()*tPose;
        Eigen::Vector2d tPt1 = mCam->Camera2Pixel(tPose1);

        MapPoint *pMp = new MapPoint(tPose, tKFrame);

        tKFrame->Add_MapPoint(pMp, i);

        pMp->Add_Observation(tKFrame, i);

        //Add MapPoint into Current Frame for image alignment and BA
        mInitFrame->mvFeatures[i]->mPoint = tPose;
        mInitFrame->Add_MapPoint(pMp);

        mMap->AddMapPoint(pMp);
    }

    int N  = tKFrame->GetVaildMpNum();
    DLOG(INFO) << N << " MapPoints created in Initialization" << endl;

    //TODO write this param into Config flie
    if(N < 100)
    {
        DLOG(ERROR) << "Initialization failed, Too few MapPoints in Initialization" << endl;

        mMap->Release();
        return false;
    }
    DLOG(INFO) << "Initialization successful!" << endl;

    //TODO add keyframe into localMapper

    return true;
}


bool Tracking::TrackWithLastFrame()
{
    mCurrentFrame->Set_Pose(mInitFrame->Get_Pose());

    int tnPts = mSprase_ImgAlign->Run(mCurrentFrame, mLastFrame);

    DLOG(INFO) << "Tracked " << tnPts << "Features" << std::endl;
    if(tnPts < 20)
    {
        mState = Lost;
        DLOG(ERROR) << "Too few feature after SpraseImage Align" << std::endl;

        return false;
    }

    return true;
}

void Tracking::TrackWithLocalMap()
{
    UpdateLocalMap();

    mFeature_Alignment->SearchLocalPoints(mCurrentFrame);

}

void Tracking::UpdateLocalMap()
{
    mFeature_Alignment->ResetGrid();

    //! Update lcoal keyframes
    std::list<std::pair<KeyFrame *, double> > tClose_kfs;
    mMap->GetCloseKeyFrames(mCurrentFrame.get(), tClose_kfs);

    //! Sort kfs refer to the distance between current frame
    tClose_kfs.sort(boost::bind(&std::pair<KeyFrame*, double>::second, _1) <
                    boost::bind(&std::pair<KeyFrame*, double>::second, _2));

    mvpLocalKeyFrames.reserve(10);

    //! Get the 10 nearest frames and Update local mappoints
    mvpLocalMapPoints.clear();

    int tNum = 0;
    for (auto iter = tClose_kfs.begin(); iter != tClose_kfs.end() && tNum<10; iter++, tNum++)
    {
        KeyFrame *tKFrame = iter->first;
        mvpLocalKeyFrames.push_back(tKFrame);


        const vector<MapPoint *> tMapPoints = tKFrame->GetMapPoints();

        for (std::vector<MapPoint *>::const_iterator itMP = tMapPoints.begin(); itMP != tMapPoints.end(); itMP++)
        {
            if ((*itMP)==NULL && (*itMP)->IsBad())
                continue;

            if ((*itMP)->mLastProjectedFrameId == mCurrentFrame->mlId)
                continue;

            (*itMP)->mLastProjectedFrameId = mCurrentFrame->mlId;

            if (mFeature_Alignment->ReprojectPoint(mCurrentFrame, (*itMP)))
                mvpLocalMapPoints.push_back((*itMP));
        }
    }
}

bool Tracking::NeedKeyframe()
{
    double tMinDepth, tMeanDepth;
    //mCurrentFrame.Get_SceneDepth(tMinDepth, tMeanDepth);
    if(mCurrentFrame->mvFeatures.size() < 30)
        return true;

    //if(mProcessedFrames<10)
    //    return false;

    Sophus::SE3 tDeltaPose = mpReferenceKF->Get_Pose().inverse()*mCurrentFrame->Get_Pose();
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
    KeyFrame *tKFrame = new KeyFrame(mCurrentFrame.get());
    mpReferenceKF = tKFrame;

    size_t tNum = 0;
    for(auto it = mCurrentFrame->mvFeatures.begin(); it!=mCurrentFrame->mvFeatures.end();it++, tNum++)
    {
        float z = mCurrentFrame->Get_FeatureDetph(mCurrentFrame->mvFeatures[tNum]->mUnpx);
        if (z<=0 && mCurrentFrame->Find_Observations(tNum))
            continue;

        Eigen::Vector3d tPose = mCurrentFrame->UnProject(mCurrentFrame->mvFeatures[tNum]->mUnpx, z);
        MapPoint *tMPoint = new MapPoint(tPose, tKFrame);

        tMPoint->Add_Observation(tKFrame, tNum);

        mCurrentFrame->Add_MapPoint(tMPoint);

        tKFrame->Add_MapPoint(tMPoint, tNum);
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
    for (size_t i = 0; i < mLastFrame->mvFeatures.size(); ++i)
    {
        mvcStatus.push_back(mLastFrame->mvFeatures[i]->mlId);
        mvsTrack_cnt.push_back(mLastFrame->mvFeatures[i]->mTrack_cnt);
    }
}

void Tracking::UpdateID(Features &features)
{
    for (int i = 0; i < features.size(); ++i)
    {
        if(features[i]->mlId==-1)
            features[i]->mlId = mlNextID++;

        features[i]->mTrack_cnt++;
    }
}

} //namespace DSDTM