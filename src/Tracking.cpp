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
    KeyFrame *tKFrame = new KeyFrame(mInitFrame);

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
        mInitFrame->Add_MapPoint(pMp, i);

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

    MotionRemovalTest1();
    //SetMpWeights();

    Optimizer::PoseOptimization(mCurrentFrame, 10);
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

        //Eigen::Vector3d tPose = mCurrentFrame->UnProject(tFeature->mpx, z);
        Eigen::Vector3d tPose = tFeature->mNormal*z;
        tPose = mCurrentFrame->Get_Pose().inverse()*tPose;

        MapPoint *tMp = new MapPoint(tPose, tKFrame);

        tKFrame->Add_MapPoint(tMp, i);

        tMp->Add_Observation(tKFrame, i);

        tFeature->SetPose(tPose);
        mCurrentFrame->Add_MapPoint(tMp, i);

        mMap->AddMapPoint(tMp);
        tNum++;
    }
    mpLastKF = tKFrame;

    DLOG(INFO)<< "Create new Keyframe " << tKFrame->mlId << " with " << tNum << " new MapPoints" <<std::endl;
}

void Tracking::MotionRemoval()
{
    int N = mCurrentFrame->mvMapPoints.size();

    Eigen::Matrix<double, 3, Eigen::Dynamic> tCurMps, tLastMps;
    tCurMps.resize(Eigen::NoChange, N);
    tLastMps.resize(Eigen::NoChange, N);

    std::vector<MapPoint*> tLastMapPoints = mLastFrame->mvMapPoints;
    Sophus::SE3 tT_c2r = mCurrentFrame->Get_Pose()*mLastFrame->Get_Pose().inverse();
    double tRelDist = tT_c2r.translation().norm();

    std::vector<cv::Point2f> tFeaturesA, tFeaturesB;
    std::vector<double> tAngles, tDistSets;
    std::vector<uchar> tStatus;

    int tNum = 0, tNum1 = 0;
    for (auto iter = mCurrentFrame->mvMapPoints.begin(); iter!=mCurrentFrame->mvMapPoints.end(); iter++, ++tNum)
    {
        std::vector<MapPoint*>::iterator result = std::find(tLastMapPoints.begin(), tLastMapPoints.end(), (*iter));
        if(result!=tLastMapPoints.end())
        {
            float z = mCurrentFrame->Get_FeatureDetph(mCurrentFrame->mvFeatures[tNum]);

            if(z < 0)
                continue;

            Eigen::Vector3d tCurMp = mCurrentFrame->mvFeatures[tNum]->mNormal*z;
            cv::Point2f tCurPx = mCurrentFrame->mvFeatures[tNum]->mpx;

            int it = result - tLastMapPoints.begin();
            Feature *tLastPt = mLastFrame->mvFeatures[it];
            Eigen::Vector3d tLastMp = tT_c2r*(tLastPt->mNormal*mLastFrame->Get_FeatureDetph(tLastPt));

            Eigen::Vector3d tMpdiff = tLastMp - tCurMp;

            cv::Point2f tLastPx = cv::Point2f(mCam->Camera2Pixel(tLastMp)(0), mCam->Camera2Pixel(tLastMp)(1));
            //cv::Point2f tLastPx = tLastPt->mpx;
            tFeaturesA.push_back(tCurPx);
            tFeaturesB.push_back(tLastPx);

            Eigen::Vector2d tFlow(tCurPx.x - tLastPx.x, tCurPx.y - tLastPx.y);
            tFlow.normalize();

            double tAngle = atan(tFlow(1)/tFlow(0));
            /*
            if(tAngle < 0)
                tAngle += 2*M_PI;
            */

            tAngles.push_back(tAngle);
            tDistSets.push_back(std::abs(tMpdiff.norm() - tRelDist));
            if(tDistSets.back() > 0.1)
                tStatus.push_back(1);
            else
                tStatus.push_back(0);

            tNum1++;
        }
    }
    mMoving_detecter->Mod_FastMCD(mCurrentFrame->mColorImg, tFeaturesA, tFeaturesB);
    //mMoving_detecter->Mod_FrameDiff(mCurrentFrame, mLastFrame, tFeaturesA, tFeaturesB);
    //mCam->Draw_Lines(mCurrentFrame->mColorImg, tFeaturesA, tFeaturesB);
    /*
    tCurMps.conservativeResize(Eigen::NoChange, tNum1);
    tLastMps.conservativeResize(Eigen::NoChange, tNum1);

    Eigen::MatrixXd tMpDiffer = tCurMps - tLastMps;
    tMpDiffer.colwise().normalize();

    cv::Mat tSamples(tNum1, 3, CV_32FC1, cv::Scalar::all(0.0));
    for (size_t i = 0; i < tNum1; ++i)
    {
        tSamples.at<float>(i, 0) = tMpDiffer(0, i);
        tSamples.at<float>(i, 1) = tMpDiffer(1, i);
        tSamples.at<float>(i, 2) = tMpDiffer(2, i);
    }
    tSamples.reshape(1, 0);
    */

    /*
    TicToc tc;
    cv::Mat tSamples(tAngles, false);
    std::vector<uchar> labels;
    cv::EM em_model;
    em_model.set("nclusters", 4);
    em_model.set("covMatType", EM::COV_MAT_SPHERICAL);
    em_model.train(tSamples, noArray(), labels, noArray());

    cv::Mat means = em_model.getMat("means");
    vector<Mat> cov = em_model.getMatVector("covs");
    double time = tc.toc();

    std::cout << means <<std::endl;
    double *tvmean = means.ptr<double>(0);
    //double taverge = tvmean[0];
    double taverge = tvmean[0];
    uchar flag = 0;
    for (int j = 1; j < 3; ++j)
    {
        if(taverge < tvmean[j])
        {
            taverge = tvmean[j];
            flag = j;
        }
    }
    //mCam->Show_Features(mCurrentFrame->mColorImg, tFeaturesA, labels, flag);

    mCam->Draw_Features(mCurrentFrame->mColorImg, tFeaturesA, tStatus);
     */
    tNum = 0;
}

void Tracking::MotionRemovalTest1()
{
    Sophus::SE3 tT_c2r = mCurrentFrame->Get_Pose()*mLastFrame->Get_Pose().inverse();

    int tRows = mCurrentFrame->mColorImg.rows/24;
    int tClos = mCurrentFrame->mColorImg.cols/32;

    std::vector<cv::Point2f> tCurPts, tLastPts;
    std::vector<double> tAngleSets;
    int n = 0;
    for (int i = 0; i < tRows; ++i)
    {
        for (int j = 0; j < tClos; ++j, ++n)
        {
            int x = j*32 + 16;
            int y = i*24 + 12;

            cv::Point2f tCurPx(x, y);

            float z = mLastFrame->Get_FeatureDetph(tCurPx);
            if(z < 0)
                continue;

            Eigen::Vector3d tPt = tT_c2r*mCam->Pixel2Camera(tCurPx, z);
            tPt = tT_c2r*tPt;
            Eigen::Vector2d tLastPx = mCam->Camera2Pixel(tPt);

            double angle = atan( (tLastPx(1) - tCurPx.y) / (tLastPx(0) - tCurPx.x) );

            tAngleSets.push_back(angle);

            tCurPts.push_back(tCurPx);
            tLastPts.push_back(cv::Point2f(tLastPx(0), tLastPx(1)));
        }
    }
    //mCam->Draw_Lines(mCurrentFrame->mColorImg, tCurPts, tLastPts);

    cv::Mat tSamples(tAngleSets, false);
    std::vector<uchar> labels;
    cv::EM em_model;
    em_model.set("nclusters", 5);
    em_model.set("covMatType", EM::COV_MAT_SPHERICAL);
    em_model.train(tSamples, noArray(), labels, noArray());

    cv::Mat means = em_model.getMat("means");
    vector<Mat> cov = em_model.getMatVector("covs");

    double *tvmean = means.ptr<double>(0);
    //double taverge = tvmean[0];
    double taverge = tvmean[0];
    uchar flag = 0;
    for (int j = 1; j < 3; ++j)
    {
        if(taverge < tvmean[j])
        {
            taverge = tvmean[j];
            flag = j;
        }
    }

    std::cout << means <<std::endl;
    mCam->Show_Features(mCurrentFrame->mColorImg, tCurPts, labels, flag);
    n = 0;
}

void Tracking::SetMpWeights()
{
    int N = mCurrentFrame->mvMapPoints.size();

    Eigen::Matrix<double, 3, Eigen::Dynamic> tCurMps, tLastMps;
    tCurMps.resize(Eigen::NoChange, N);
    tLastMps.resize(Eigen::NoChange, N);

    std::vector<MapPoint*> tLastMapPoints = mLastFrame->mvMapPoints;

    Sophus::SE3 tT_c2w = mCurrentFrame->Get_Pose();
    Sophus::SE3 tT_c2r = tT_c2w*mLastFrame->Get_Pose().inverse();

    std::vector<MapPoint*> tUpdataMps;
    std::vector<double> tDistSets;
    int tNum = 0, tNum1 = 0;
    for (auto iter = mCurrentFrame->mvMapPoints.begin(); iter!=mCurrentFrame->mvMapPoints.end(); iter++, ++tNum)
    {
        std::vector<MapPoint*>::iterator result = std::find(tLastMapPoints.begin(), tLastMapPoints.end(), (*iter));
        if(result!=tLastMapPoints.end())
        {
            Eigen::Vector3d tCurMp = mCurrentFrame->mvFeatures[tNum]->mNormal*mCurrentFrame->Get_FeatureDetph(mCurrentFrame->mvFeatures[tNum]);

            int it = result - tLastMapPoints.begin();
            Feature *tLastPt = mLastFrame->mvFeatures[it];
            Eigen::Vector3d tMpdiff = tT_c2r*(tLastPt->mNormal*mLastFrame->Get_FeatureDetph(tLastPt)) - tCurMp;

            tDistSets.push_back(tMpdiff.norm());
            tUpdataMps.push_back((*iter));

            tNum1++;
        }
    }

    std::vector<double> Weights;
    mCam->VarWithMAD(tDistSets, &Weights);
    tNum = 0;
    for (auto it = tUpdataMps.begin(); it!=tUpdataMps.end(); ++it, ++tNum)
    {
        (*it)->SetStaticWeight(Weights[tNum]);
    }

    mCam->Draw_Features(mCurrentFrame->mColorImg, mCurrentFrame->mvFeatures);
    std::cout << "Finish" << std::endl;
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