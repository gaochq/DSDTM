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
                LOG(ERROR) << "Tracking lost with last frame" << std::endl;

            if(bOK)
            {
                mMapDrawer->SetCurrentCameraPose(mCurrentFrame->Get_Pose());

                if (NeedKeyframe())
                {
                    CraeteKeyframe();

                    //if(mMap->ReturnKeyFramesSize() > 2)
                        //Optimizer::LocalBundleAdjustment(mpLastKF, mMap);
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
        //Eigen::Vector3d tPose = mInitFrame->mvFeatures[i]->mNormal*z;
        //tPose = mInitFrame->Get_Pose().inverse()*tPose;

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

    int N = mCurrentFrame->mvFeatures.size();
    LOG(INFO)<< mCurrentFrame->mlId <<" Frame tracked " << N << " Features" << std::endl;

    //MotionRemoval();

    mCam->Draw_Features(mCurrentFrame->mColorImg, mCurrentFrame->mvFeatures);

    Optimizer::PoseOptimization(mCurrentFrame, 10);

    if(N < 30)
    {
        LOG(WARNING)<< "Too few Features tracked" << std::endl;

        if(N < 20)
            return false;

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
    //! This condition frome HeYijia / svo_edgelet
    //! https://github.com/HeYijia/svo_edgelet/blob/master/src/frame_handler_mono.cpp#L501
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
    double d = mCam->GetMedian(tPixDist);
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
        if(fabs(tRelPose(0)/tMeanDepth < tMeanDepth) &&
           fabs(tRelPose(1)/tMeanDepth < tMeanDepth*0.8) &&
           fabs(tRelPose(2)/tMeanDepth < tMeanDepth*1.3))
            return false;
    }

    //! More than 20 frames passed after last KeyFrame insertion
    if(mProcessedFrames > 20)
        return true;

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

    mMoving_detecter->Mod_FastMCD(mCurrentFrame->mColorImg, tFeaturesA, tFeaturesB);
}

void Tracking::MotionRemovalTest1()
{
    Sophus::SE3 tPose1 = Sophus::SE3(Eigen::Quaterniond(0.6388, -0.7682, -0.0432, 0.0006 ), Eigen::Vector3d(-0.6518, -3.1123, 1.4229)).inverse();
    Sophus::SE3 tPose2 = Sophus::SE3(Eigen::Quaterniond(0.6388, -0.7680, -0.0452, 0.0010), Eigen::Vector3d(-0.6600, -3.1136, 1.4237)).inverse();
    Sophus::SE3 tT_c2r = tPose1*tPose2.inverse();
    //Sophus::SE3 tT_c2r = mCurrentFrame->Get_Pose()*mLastFrame->Get_Pose().inverse();

    int tRows = mCurrentFrame->mColorImg.rows/24;
    int tClos = mCurrentFrame->mColorImg.cols/32;

    std::vector<cv::Point2f> tCurPts, tLastPts;
    std::vector<float> tAngleSets;
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

            Eigen::Vector3d tPt = mCam->Pixel2Camera(tCurPx, z);
            tPt = tT_c2r*tPt;
            Eigen::Vector2d tLastPx = mCam->Camera2Pixel(tPt);

            float angle = atan( (tLastPx(1) - tCurPx.y) / (tLastPx(0) - tCurPx.x) );
            if(angle < 0)
                angle += 1*M_PI;

            tAngleSets.push_back(angle);


            tCurPts.push_back(cv::Point2f(tLastPx(0), tLastPx(1)));
            tLastPts.push_back(tCurPx);
        }
    }
    mCam->Draw_Lines(mCurrentFrame->mColorImg, tCurPts, tLastPts);
    //mMoving_detecter->Mod_FrameDiff(mCurrentFrame, mLastFrame, tCurPts, tLastPts);

    /*
    // Kmeans
    Mat points=Mat(tAngleSets, false);
    points.convertTo(points, CV_32FC1);
    //points.reshape(1);

    cv::Mat labels;
    cv::Mat centers;
    cv::Mat Points = cv::Mat(tAngleSets).reshape(1, tAngleSets.size());
    cv::kmeans(Points, 3, labels, cv::TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
    std::vector<uchar> tLabels;
    for (int k = 0; k < tAngleSets.size(); ++k)
    {
        uchar clusterIdx = labels.at<int>(k);

        tLabels.push_back(clusterIdx);
    }
    std::cout << centers <<std::endl;
     mCam->Show_Features(mCurrentFrame->mColorImg, tCurPts, tLabels, 1);
     */


    cv::Mat tSamples(tAngleSets, false);
    std::vector<uchar> labels;
    cv::EM em_model;
    em_model.set("nclusters", 4);
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

void Tracking::MotionRemovalTest2()
{
    cv::Mat imageA = mCurrentFrame->mColorImg;
    cv::Mat imageB = mLastFrame->mColorImg;

    vector<cv::KeyPoint> KeyPointA, KeyPointB;
    cv::ORB orb;

    orb.detect(imageA, KeyPointA);
    orb.detect(imageB, KeyPointB);

    cv::Mat DescriptorsA, DescriptorsB;
    orb.compute(imageA, KeyPointA, DescriptorsA);
    orb.compute(imageB, KeyPointB, DescriptorsB);

    // match the keypoints
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    vector<cv::DMatch> matches, Good_matches;
    matcher.match(DescriptorsA, DescriptorsB, matches);

    double mini_dist = 1000, max_dist = 0;
    for (int i = 0; i < DescriptorsA.rows ; ++i)
    {
        double dist = matches[i].distance;
        if(dist < mini_dist)  mini_dist = dist;
        if(dist > max_dist)     max_dist =dist;
    }

    for (int j = 0; j < DescriptorsA.rows; ++j)
    {
        if(matches[j].distance <= max(2*mini_dist, 30.0))
            Good_matches.push_back(matches[j]);
    }

    vector<int> PointIndexA, PointIndexB;
    for (int i = 0; i < Good_matches.size(); ++i)
    {
        cv::DMatch it = Good_matches[i];
        PointIndexA.push_back(it.queryIdx);
        PointIndexB.push_back(it.trainIdx);
    }

    cv::vector<cv::Point2f> PointsA, PointsB;
    cv::KeyPoint::convert(KeyPointA, PointsA, PointIndexA);
    cv::KeyPoint::convert(KeyPointB, PointsB, PointIndexB);

    Sophus::SE3 tT_c2r = mCurrentFrame->Get_Pose()*mLastFrame->Get_Pose().inverse();

    std::vector<double> tAngleSets;
    std::vector<cv::Point2f> tFeaturesA, tFeaturesB;
    for (int k = 0; k < PointsA.size(); ++k)
    {
        float z = mLastFrame->Get_FeatureDetph(PointsB[k]);
        if(z < 0)
            continue;

        Eigen::Vector3d tLastPt = mCam->Pixel2Camera(PointsB[k], 1.0);
        tLastPt.normalize();
        tLastPt = tT_c2r*(tLastPt*z);
        Eigen::Vector2d t_C2LPx = mCam->Camera2Pixel(tLastPt);

        double angle = atan( (t_C2LPx(1) - PointsA[k].y)/(t_C2LPx(0) - PointsA[k].x) );
        if(angle < 0 )
            angle += M_PI;
        tAngleSets.push_back(angle);

        tFeaturesA.push_back(PointsA[k]);
        tFeaturesB.push_back(cv::Point2f(t_C2LPx(0), t_C2LPx(1)));
    }

    mCam->Draw_Lines(mCurrentFrame->mColorImg, tFeaturesA, tFeaturesB);

    cv::Mat tSamples(tAngleSets, false);
    std::vector<uchar> labels;
    cv::EM em_model;
    em_model.set("nclusters", 4);
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
    mCam->Show_Features(mCurrentFrame->mColorImg, tFeaturesA, labels, flag);


    mini_dist = 0;
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