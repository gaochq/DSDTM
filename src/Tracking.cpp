//
// Created by buyi on 17-10-19.
//

#include "Tracking.h"



namespace DSDTM
{

long int Tracking::mlNextID = 0;

Tracking::Tracking(CameraPtr _cam, Map *_map, LocalMapping *tLocalMapping, MapDrawer *tMapDrawer):
        mCam(_cam), mInitializer(static_cast<Initializer*>(NULL)), mMap(_map), mLocalMapping(tLocalMapping),
        mProcessedFrames(0), mMapDrawer(tMapDrawer)
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

            mMapDrawer->SetCurrentCameraPose(mCurrentFrame->Get_Pose());

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
                DLOG(ERROR) << "Tracking lost with last frame" << std::endl;

            if(bOK)
            {
                mMapDrawer->SetCurrentCameraPose(mCurrentFrame->Get_Pose());

                if (NeedKeyframe())
                    CraeteKeyframe();
            }
            else
            {
                DLOG(ERROR) << "Trakcing lost in LocalMap" << std::endl;
            }

            mProcessedFrames++;
        }

    }

    mLastFrame = mCurrentFrame;

    Reset_Status();

    return mCurrentFrame->Get_Pose().inverse();
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

        MapPoint *pMp = new MapPoint(tPose, tKFrame);

        tKFrame->Add_MapPoint(pMp, i);

        pMp->Add_Observation(tKFrame, i);

        //Add MapPoint into Current Frame for image alignment and BA
        mInitFrame->mvFeatures[i]->SetPose(tPose);
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

    mpLastKF = tKFrame;

    //TODO add keyframe into localMapper

    return true;
}


bool Tracking::TrackWithLastFrame()
{
    mCurrentFrame->Set_Pose(mLastFrame->Get_Pose());

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

bool Tracking::TrackWithLocalMap()
{
    UpdateLocalMap();

    mFeature_Alignment->SearchLocalPoints(mCurrentFrame);

    int N = mCurrentFrame->mvFeatures.size();
    DLOG(INFO)<< mCurrentFrame->mlId <<" Frame tracked " << N << " Features" << std::endl;

    //Optimizer::PoseOptimization(mCurrentFrame, 10);
    double inialError, finalError;
    //Optimizer::optimizeGaussNewton(2.0, 10, mCurrentFrame, inialError, finalError);
    if(N < 30)
    {
        DLOG(ERROR)<< "Too few Features tracked" << std::endl;
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
                mvpLocalMapPoints.push_back((*itMP));
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
    if(mCurrentFrame->mvFeatures.size() < 50)
        return true;

    //! This condition frome HeYijia / svo_edgelet
    //! https://github.com/HeYijia/svo_edgelet/blob/master/src/frame_handler_mono.cpp#L501
    std::vector<double> tPixDist;
    for (auto it = mpLastKF->mvFeatures.begin(); it !=mpLastKF->mvFeatures.end() ; ++it)
    {
        if((*it)->mPoint.isZero())
            continue;

        Eigen::Vector2d tPix1 = mCurrentFrame->World2Pixel((*it)->mPoint);
        Eigen::Vector2d tPix2 = mpLastKF->World2Pixel((*it)->mPoint);
        Eigen::Vector2d tPixDiff = tPix1 - tPix2;

        tPixDist.push_back(tPixDiff.norm());

        if(tPixDist.size() > 30)
            break;
    }
    double d = mCam->GetMedian(tPixDist);
    if(d > 40)
    {
        return true;
    }

    //if(mProcessedFrames<10)
    //    return false;

    Sophus::SE3 tDeltaPose = mpLastKF->Get_Pose().inverse()*mCurrentFrame->Get_Pose();
    double tRotNorm = tDeltaPose.so3().log().norm();
    double tTransNorm = tDeltaPose.translation().norm();
    if(tRotNorm >= mdMinRotParallax && tTransNorm >= mdMinTransParallax)
    {
        return true;
    }

    //! This condition from SVO, and only consider translation in keyframe selection
    double tMinDepth, tMeanDepth;
    mCurrentFrame->Get_SceneDepth(tMinDepth, tMeanDepth);
    for (auto it = mvpLocalKeyFrames.begin(); it!=mvpLocalKeyFrames.end() ; ++it)
    {
        Eigen::Vector3d tRelPose = mCurrentFrame->Get_Pose()*(*it)->Get_CameraCnt();
        if(fabs(tRelPose(0)/tMeanDepth < tMeanDepth) &&
           fabs(tRelPose(1)/tMeanDepth < tMeanDepth*0.8) &&
           fabs(tRelPose(2)/tMeanDepth < tMeanDepth*1.3))
            return false;
    }

    return true;
}

void Tracking::CraeteKeyframe()
{
    //! Add new Features
    mFeature_detector->Set_ExistingFeatures(mCurrentFrame->mvFeatures);
    mFeature_detector->detect(mCurrentFrame.get(), 20);
    mCurrentFrame->UndistortFeatures();

    KeyFrame *tKFrame = new KeyFrame(mCurrentFrame.get());
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

        //Eigen::Vector3d tPose = mCurrentFrame->UnProject(tFeature->mpx, z);
        Eigen::Vector3d tPose = tFeature->mNormal*z;
        tPose = mCurrentFrame->Get_Pose().inverse()*tPose;

        MapPoint *tMp = new MapPoint(tPose, tKFrame);

        tKFrame->Add_MapPoint(tMp, i);

        tMp->Add_Observation(tKFrame, i);

        tFeature->SetPose(tPose);
        mCurrentFrame->Add_MapPoint(tMp);

        mMap->AddMapPoint(tMp);
        tNum++;
    }
    mpLastKF = tKFrame;

    DLOG(INFO)<< "Create new Keyframe " << tKFrame->mlId << " with " << tNum << " new MapPoints" <<std::endl;
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