//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"



namespace DSDTM
{

long int Tracking::mlNextID = 0;

Tracking::Tracking(CameraPtr _cam, Map *_map, LocalMapping *tLocalMapping):
        mCam(_cam), mInitializer(static_cast<Initializer*>(NULL)), mMap(_map), mLocalMapping(tLocalMapping),
        mProcessedFrames(0)
{
    mState = Not_Init;

    mMaxPyra_levels = Config::Get<int>("Camera.MaxPyraLevels");
    mMinPyra_levels = Config::Get<int>("Camera.MinPyraLevels");

    mDepthScale = Config::Get<float>("Camera.depth_scale");
    mMaxIters = Config::Get<int>("Optimization.MaxIter");

    mdMinRotParallax = Config::Get<double>("KeyFrame.min_rot");
    mdMinTransParallax = Config::Get<double>("KeyFrame.min_trans");
    mMinFeatures = Config::Get<int>("KeyFrame.min_features");
    mdMinDist = Config::Get<int>("KeyFrame.min_dist");

    mFeature_detector = new Feature_detector();

    mMoving_detecter = new Moving_Detecter();

    mFeature_Alignment = new Feature_Alignment(mCam);

    mSprase_ImgAlign = new Sprase_ImgAlign(mMaxPyra_levels, mMinPyra_levels, mMaxIters);
}

Tracking::~Tracking()
{

}

Sophus::SE3 Tracking::Track_RGBDCam(const cv::Mat &colorImg, const cv::Mat &depthImg, const double ctimestamp)
{
    cv::Mat tDImg = depthImg;
    if(!colorImg.data || !depthImg.data)
    {
        std::cout<< "No images" <<std::endl;

        mState = No_Images;
        return IdentitySE3;
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
                return IdentitySE3;

            mViewer->SetCurrentCameraPose(mCurrentFrame->Get_Pose());

            mState = OK;
            mCurrentFrame->Reset_Gridproba();


            mlNextID = mInitFrame->mvFeatures.size();
        }

        //std::cout << mCurrentFrame->Get_CameraCnt().transpose() <<std::endl;
    }
    else 
    {
        if(mState==OK)
        {
            bool bOK;

            //TicToc tc;
            bOK = TrackWithLastFrame();
            //std::cout << tc.toc() <<"----";

            //std::cout << mCurrentFrame->Get_CameraCnt().transpose() <<std::endl;

            if(bOK)
            {
                bOK = TrackWithLocalMap();
                //std::cout << tc.toc() << std::endl;
            }
            else
                LOG(ERROR) << "Tracking lost with last frame" << std::endl;

            if(bOK)
            {
                mViewer->SetCurrentCameraPose(mCurrentFrame->Get_Pose());

                if (NeedKeyframe())
                {
                    CraeteKeyframe();

                    if(mMap->ReturnKeyFramesSize() > 2)
                        Optimizer::LocalBundleAdjustment(mpLastKF, mMap);
                }

            }
            else
            {
                LOG(ERROR) << "Trakcing lost in LocalMap" << std::endl;
            }

            mProcessedFrames++;
        }

    }

    mLastFrame = mCurrentFrame;

    Reset_Status();

    mTrajectory.push_back(std::make_pair(mCurrentFrame->mdCloTimestamp, mCurrentFrame->Get_Pose()));

    return mCurrentFrame->Get_Pose().inverse();
}

bool Tracking::CreateInitialMapRGBD()
{
    mInitFrame->UndistortFeatures();
    KeyFrame *tKFrame = new KeyFrame(mInitFrame);

    //TODO detect orb Feature and compute the BOW vector

    mMap->AddKeyFrame(tKFrame);

    for (int i = 0; i < mInitFrame->mvFeatures.size(); ++i)
    {
        float z = mInitFrame->Get_FeatureDetph(mInitFrame->mvFeatures[i]->mpx);

        if(z < 0)
            continue;

        Eigen::Vector3d tPose = mInitFrame->UnProject(mInitFrame->mvFeatures[i]->mpx, z);

        MapPoint *pMp = new MapPoint(tPose, tKFrame, mMap);

        tKFrame->Add_MapPoint(pMp, i);

        pMp->Add_Observation(tKFrame, i);

        //Add MapPoint into Current Frame for image alignment and BA
        mInitFrame->mvFeatures[i]->SetPose(pMp);
        mInitFrame->Add_MapPoint(pMp, i);

        mMap->AddMapPoint(pMp);
    }

    int N  = tKFrame->GetVaildMpNum();
    LOG(INFO) << N << " MapPoints created in Initialization" << endl;

    //TODO write this param into Config flie
    if(N < 100)
    {
        LOG(ERROR) << "Initialization failed, Too few MapPoints in Initialization" << endl;

        mMap->Release();
        return false;
    }
    LOG(INFO) << "Initialization successful!" << endl;

    mpLastKF = tKFrame;

    //TODO add keyframe into localMapper

    return true;
}


bool Tracking::TrackWithLastFrame()
{
    mCurrentFrame->Set_Pose(mLastFrame->Get_Pose());

    int tnPts = mSprase_ImgAlign->Run(mCurrentFrame, mLastFrame);


    LOG(INFO) << "Tracked " << tnPts << "Features" << std::endl;
    if(tnPts < 20)
    {
        mState = Lost;
        LOG(ERROR) << "Too few feature after SpraseImage Align" << std::endl;

        return false;
    }

    return true;
}

bool Tracking::TrackWithLocalMap()
{
    UpdateLocalMap();

    mFeature_Alignment->SearchLocalPoints(mCurrentFrame);

    mCurrentFrame->mvFeatures.size();

    MotionRemoval();
    //MotionRemovalTest1();

    mCam->Draw_Features(mCurrentFrame->mColorImg, mCurrentFrame->mvFeatures);

    Optimizer::PoseOptimization(mCurrentFrame, 10);

    int N = mCurrentFrame->Get_VaildMpNums();

    LOG(INFO)<< mCurrentFrame->mlId <<" Frame tracked " << N << " Features" << std::endl;

    std::cout << "Tracking Pts: " << N << std::endl;

    if(N < 30)
    {
        LOG(WARNING)<< "Too few Features tracked" << std::endl;

        //if(N < 10)
        //    return false;

        return true;
    }
    else
        return true;

}

void Tracking::UpdateLocalMap()
{
    mFeature_Alignment->ResetGrid();

    //! Update lcoal keyframes
    std::list<std::pair<KeyFrame *, double> > tClose_kfs;
    GetCloseKeyFrames(mCurrentFrame.get(), tClose_kfs);

    //! Sort kfs refer to the distance between current frame
    tClose_kfs.sort(boost::bind(&std::pair<KeyFrame*, double>::second, _1) <
                    boost::bind(&std::pair<KeyFrame*, double>::second, _2));

    mvpLocalKeyFrames.clear();
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
            if ((*itMP)==NULL)
                continue;

            if((*itMP)->IsBad())
                continue;

            if ((*itMP)->mLastProjectedFrameId == mCurrentFrame->mlId)
                continue;

            (*itMP)->mLastProjectedFrameId = mCurrentFrame->mlId;

            if (mFeature_Alignment->ReprojectPoint(mCurrentFrame, (*itMP)))
                mvpLocalMapPoints[(*itMP)] = tKFrame;
        }
    }
}

void Tracking::GetCloseKeyFrames(const Frame *tFrame, std::list<std::pair<KeyFrame *, double> > &tClose_kfs) const
{
    std::vector<KeyFrame*> tvKeyFrames = mMap->GetAllKeyFrames();
    //TODO improve the stragedy to choose the local keyframes

    for (auto kf = tvKeyFrames.begin(); kf!= tvKeyFrames.end(); ++kf)
    {
        for (auto keypoint = (*kf)->mvMapPoints.begin(); keypoint!=(*kf)->mvMapPoints.end(); ++keypoint)
        {
            if(!(*keypoint))
                continue;

            if((*keypoint)->Get_Pose().isZero(0))
                continue;

            if(tFrame->isVisible((*keypoint)->Get_Pose()))
            {
                tClose_kfs.push_back(
                        std::make_pair((*kf), (tFrame->Get_Pose().translation() - (*kf)->Get_Pose().translation()).norm()));
                break;
            }
        }
    }
}

bool Tracking::NeedKeyframe()
{
    //! This condition frome HeYijia / svo_edgelet
    //! https://github.com/HeYijia/svo_edgelet/blob/master/src/frame_handler_mono.cpp#L501

    int N = mCurrentFrame->Get_VaildMpNums();
    if(N >= 10 && N <= 50)
    {
        return true;
    }

    std::vector<double> tPixDist;
    for (auto it = mpLastKF->mvFeatures.begin(); it !=mpLastKF->mvFeatures.end() ; ++it)
    {
        if(!(*it)->Mpt)
            continue;

        if((*it)->Mpt->IsBad())
            continue;

        Eigen::Vector2d tPix1 = mCurrentFrame->World2Pixel((*it)->Mpt->Get_Pose());
        Eigen::Vector2d tPix2 = mpLastKF->World2Pixel((*it)->Mpt->Get_Pose());
        Eigen::Vector2d tPixDiff = tPix1 - tPix2;

        tPixDist.push_back(tPixDiff.norm());

        if(tPixDist.size() > 30)
            break;
    }
    double d = utils::GetMedian(tPixDist);
    if(d > 40)
    {
        return true;
    }

    //! If the translation and rotation difference larger than a threshold
    Sophus::SE3 tDeltaPose = mpLastKF->Get_Pose().inverse()*mCurrentFrame->Get_Pose();
    double tRotNorm = tDeltaPose.so3().log().norm();
    double tTransNorm = tDeltaPose.translation().norm();
    if(tRotNorm >= mdMinRotParallax || tTransNorm >= mdMinTransParallax)
    {
        return true;
    }

    //! This condition from SVO, and only consider translation in keyframe selection
    double tMinDepth, tMeanDepth;
    mCurrentFrame->Get_SceneDepth(tMinDepth, tMeanDepth);
    for (auto it = mvpLocalKeyFrames.begin(); it!=mvpLocalKeyFrames.end() ; ++it)
    {
        Eigen::Vector3d tRelPose = mCurrentFrame->Get_Pose()*(*it)->Get_CameraCnt();
        if(fabs(tRelPose(0)/tMeanDepth >= tMeanDepth) &&
           fabs(tRelPose(1)/tMeanDepth >= tMeanDepth*0.8) &&
           fabs(tRelPose(2)/tMeanDepth >= tMeanDepth*1.3))
            return true;
    }

    //! More than 20 frames passed after last KeyFrame insertion
    //if(mProcessedFrames > 20)
    //    return true;


    return false;
}

void Tracking::CraeteKeyframe()
{
    //! Add new Features
    mFeature_detector->Set_ExistingFeatures(mCurrentFrame->mvFeatures);
    mFeature_detector->detect(mCurrentFrame.get(), 5.0);
    mCurrentFrame->UndistortFeatures();


    KeyFrame *tKFrame = new KeyFrame(mCurrentFrame);
    mMap->AddKeyFrame(tKFrame);

    int tNum = 0;
    for (int i = 0; i < mCurrentFrame->mvFeatures.size(); ++i)
    {
        Feature *tFeature = mCurrentFrame->mvFeatures[i];

        if(tFeature->mbInitial)
        {
            tKFrame->Add_MapPoint(mCurrentFrame->mvMapPoints[i], i);
            mCurrentFrame->mvMapPoints[i]->Add_Observation(tKFrame, i);

            continue;
        }

        float z = mCurrentFrame->Get_FeatureDetph(tFeature->mpx);
        if(z < 0)
            continue;

        Eigen::Vector3d tPose = mCurrentFrame->UnProject(tFeature->mpx, z);
        //Eigen::Vector3d tPose = tFeature->mNormal*z;
        //tPose = mCurrentFrame->Get_Pose().inverse()*tPose;

        MapPoint *tMp = new MapPoint(tPose, tKFrame, mMap);

        tKFrame->Add_MapPoint(tMp, i);

        tMp->Add_Observation(tKFrame, i);

        tFeature->SetPose(tMp);
        mCurrentFrame->Add_MapPoint(tMp, i);

        mMap->AddMapPoint(tMp);
        tNum++;
    }
    mCam->Draw_Features(mCurrentFrame->mColorImg, mCurrentFrame->mvFeatures);
    mpLastKF = tKFrame;

    tKFrame->UpdateConnection();

    mProcessedFrames = 0;

    LOG(INFO)<< "Create new Keyframe " << tKFrame->mlId << " with " << tNum << " new MapPoints" <<std::endl;
}

void Tracking::MotionRemoval()
{
    int N = mCurrentFrame->mvMapPoints.size();

    std::vector<MapPoint*> tLastMapPoints = mLastFrame->mvMapPoints;
    std::vector<cv::Point2f> tFeaturesA, tFeaturesB;

    int tNum = 0;
    for (auto iter = mCurrentFrame->mvMapPoints.begin(); iter!=mCurrentFrame->mvMapPoints.end(); iter++, ++tNum)
    {
        std::vector<MapPoint*>::iterator result = std::find(tLastMapPoints.begin(), tLastMapPoints.end(), (*iter));
        if(result!=tLastMapPoints.end())
        {
            tFeaturesA.push_back(mCurrentFrame->mvFeatures[tNum]->mpx);

            int n = result - tLastMapPoints.begin();
            tFeaturesB.push_back(mLastFrame->mvFeatures[n]->mpx);
        }
    }

    cv::Mat tMask = mMoving_detecter->Mod_FastMCD(mCurrentFrame->mColorImg, tFeaturesA, tFeaturesB);
    //cv::Mat tMask = mMoving_detecter->Mod_FrameDiff(mCurrentFrame, mLastFrame, tFeaturesA, tFeaturesB);
    if(tMask.rows!=480)
        return;
    mCurrentFrame->Motion_Removal(tMask);
}

void Tracking::MotionRemovalTest1()
{
    std::vector<MapPoint*> tMpSets = mCurrentFrame->mvMapPoints;
    std::vector<MapPoint*> tRefMpSets = mpLastKF->mvMapPoints;
    std::vector<MapPoint*> tMatchMps;
    std::vector<double> tResiduals;

    Sophus::SE3 tRelPose = mCurrentFrame->Get_Pose()*mpLastKF->Get_Pose().inverse();

    for (int i = 0; i < tMpSets.size(); ++i)
    {
        auto iter = std::find(tRefMpSets.begin(), tRefMpSets.end(), tMpSets[i]);

        if(iter!=tRefMpSets.end())
        {
            int n = iter - tRefMpSets.begin();
            Eigen::Vector3d tDiff = mCurrentFrame->mvFeatures[i]->mNormal - tRelPose*mpLastKF->mvFeatures[n]->mNormal;
            tResiduals.push_back(tDiff.norm());
            tMatchMps.push_back(*iter);
        }
    }

    int N = 0;
    std::vector<double> tWeights;
    mCam->VarWithMAD(tResiduals, &tWeights);
    for (int j = 0; j < tMatchMps.size(); ++j)
    {
        double tmp = tMatchMps[j]->mdStaticWeight;
        tMatchMps[j]->mdStaticWeight = 0.5*tmp + 0.5*tWeights[j];
        tWeights[j] = tMatchMps[j]->mdStaticWeight;
        if(tWeights[j] < 1)
            N++;
    }

    //std::cout << N <<std::endl;
    //std::cout << std::endl;
}

void Tracking::SetViewer(Viewer *tViewer)
{
    mViewer = tViewer;
}

void Tracking::Reset()
{
    mViewer->RequestStop();

    while(!mViewer->IsStopped())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    // TODO request to reset localmapper

    mMap->Release();

    KeyFrame::mlNextId = 0;
    Frame::mlNextId = 0;
    MapPoint::mlNextId = 0;
    mState = Not_Init;

    if(mInitializer)
    {
        delete mInitializer;
        mInitializer = static_cast<Initializer*>(NULL);
    }

    mViewer->Release();
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