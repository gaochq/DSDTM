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

        /*
        if(mState==OK)
        {
            TrackWithReferenceFrame();

            if(NeedKeyframe())
                CraeteKeyframe();


            mProcessedFrames++;
        }
        */

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
        //Show_Features(Last_Pts);
        LKT_Track(tLast_Pts, tCur_Pts);


        //Rarsac_F(tCur_Pts, tLast_Pts, tBad_PtsA);
        cv::Mat F = Reject_FMat(tLast_Pts, tCur_Pts, tBad_PtsA);

        tBad_PtsA.clear(); tBad_PtsB.clear();
        //LKT_outlier(tCur_Pts, tLast_Pts, tBad_PtsA, tBad_PtsB, F);

        cv::Mat H = cv::findHomography(tCur_Pts, tLast_Pts, CV_RANSAC);
        mMoving_detecter->Run(mCurrentFrame.mColorImg, H.ptr<double>(0));

        std::cout << " --- " << tCur_Pts.size() << std::endl;

        if (tCur_Pts.size() < 20)
        {
            mState = Lost;
            std::cout << "Too few features: " << tCur_Pts.size() << " after rarsac" << std::endl;
            //return;
        }
    }
    //mCam->Draw_Lines(mCurrentFrame.mColorImg, tCur_Pts, tLast_Pts, tBad_PtsA, tBad_PtsB);

    AddNewFeatures(tCur_Pts, tBad_PtsA);
    mCurrentFrame.Get_Features(tPts_tmp);
    std::vector<cv::Point2f> tvNewFeatures(tPts_tmp.begin()+tCur_Pts.size(), tPts_tmp.end());
    std::vector<cv::Point2f> tvGoodFeatures(tPts_tmp.begin(), tPts_tmp.begin()+tCur_Pts.size());
    mCam->Show_Features(mCurrentFrame.mColorImg, tBad_PtsA, tvGoodFeatures, tvNewFeatures);
    //std::cout<< mCurrentFrame.mvFeatures.size() << std::endl;
    //std::cout << " --- " << tPts_tmp.size() << std::endl;
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

void Tracking::AddNewFeatures(std::vector<cv::Point2f> &tCur_Pts, std::vector<cv::Point2f> &tLast_Pts)
{
    std::vector<cv::Point2f>tPts_tmp, tBad_Pts;

    mCurrentFrame.SetFeatures(tCur_Pts, mvcStatus, mvsTrack_cnt);
    UpdateID(mCurrentFrame.mvFeatures);
    mCurrentFrame.Set_Mask(mvcStatus, mvsTrack_cnt, tLast_Pts);

    tCur_Pts.clear();
    mCurrentFrame.Get_Features(tCur_Pts);
    //mFeature_detector->Set_ExistingFeatures(mLastFrame.mvFeatures);
    //mFeature_detector->Set_ExistingFeatures(mCurrentFrame.mvFeatures);
    mFeature_detector->detect(&mCurrentFrame, 10);
}

void Tracking::TrackWithReferenceFrame()
{
    mCurrentFrame.Set_Pose(mLastFrame.Get_Pose());
    mCurrentFrame.Add_Observations(*mpReferenceKF);

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

    size_t tFeature_Num = 0;
    for(auto it = mCurrentFrame.mvFeatures.begin(); it!=mCurrentFrame.mvFeatures.end();it++)
    {
        float z = mCurrentFrame.Get_FeatureDetph(*it);
        if (z<=0 && mCurrentFrame.Find_Observations(tFeature_Num))
            continue;

        Eigen::Vector3d tPose = mCurrentFrame.UnProject(it->mpx, z);
        MapPoint *tMPoint = new MapPoint(tPose, tKFrame);

        tMPoint->Add_Observation(tKFrame, tFeature_Num);

        mCurrentFrame.Add_MapPoint(tFeature_Num, tMPoint);

        tKFrame->Add_MapPoint(tMPoint);
        tKFrame->Add_Observations(tFeature_Num, tMPoint);

        mMap->AddMapPoint(tMPoint);
        tFeature_Num++;
    }


}

void Tracking::LKT_outlier(std::vector<cv::Point2f> &tFeaturesA, std::vector<cv::Point2f> &tFeaturesB,
                           std::vector<cv::Point2f> &tBadPtsA, std::vector<cv::Point2f> &tBadPtsB, const cv::Mat F)
{
    cv::Mat H = cv::findHomography(tFeaturesB, tFeaturesA, CV_RANSAC);
    std::vector<cv::Point2f> tCompensate;
    double *h = H.ptr<double>(0);
    for (int m = 0; m < tFeaturesB.size(); ++m)
    {
        float newW = h[6] * tFeaturesB[m].x + h[7] * tFeaturesB[m].y + h[8];
        float newX = (h[0] * tFeaturesB[m].x + h[1] * tFeaturesB[m].y + h[2]) / newW;
        float newY = (h[3] * tFeaturesB[m].x + h[4] * tFeaturesB[m].y + h[5]) / newW;

        tCompensate.push_back(cv::Point2f(newX, newY));
    }


    std::vector<uchar> tvStatus;
    double tDistAve, tOrientAve;
    Eigen::Vector2d tOrient;

    std::vector<double> tvdDists, tOrients, tTanSets, tAngleSets, tvdFDists, tvdVectorx, tvdVectory;
    std::vector<Eigen::Vector2d> tvdOrients, tvLKTVectors;

    size_t N = tFeaturesA.size();
    tvStatus.resize(N, 1);

    const int HISTO_LENGTH = 6;
    const double factor = 1.0f/HISTO_LENGTH;
    std::vector<int> rotHist[HISTO_LENGTH], tvCommonVector;

    for (int i = 0; i < N; ++i)
    {
        Eigen::Vector2d tLKTVector;
        Eigen::Vector3d tPoint;

        tLKTVector << tFeaturesA[i].x - tCompensate[i].x,
                    tFeaturesA[i].y - tCompensate[i].y;

        /*
        float z1 = mCurrentFrame.Get_FeatureDetph(tFeaturesA[i]);
        tPoint = mCam->Pixel2Camera(tFeaturesA[i], z1);
        float z2 = mLastFrame.Get_FeatureDetph(tFeaturesB[i]);
        tPoint = tPoint- mCam->Pixel2Camera(tFeaturesB[i], z2);
        */

        //tvLKTVectors.push_back(tLKTVector);
        //tvdDists.push_back(tLKTVector.norm());
        //tDistAve += tvdDists.back()/N;

        tLKTVector.normalize();
        tvdOrients.push_back(tLKTVector);

        double tAngle = atan(tLKTVector[1]/tLKTVector[0]);


        //if(tAngle<0)
        //    tAngle = tAngle + 2*M_PI;
        tAngleSets.push_back(tAngle);
        /*
        int bin = round(tAngle);
        if(bin == HISTO_LENGTH)
            bin = 0;
        assert(bin>=0 && bin<HISTO_LENGTH);
        rotHist[bin].push_back(i);


        tvdVectorx.push_back(tLKTVector[0]);
        tvdVectory.push_back(tLKTVector[1]);


        //tOrient +=  tLKTVector;

        double A = F.at<double>(0, 0)*tFeaturesA[i].x + F.at<double>(0, 1)*tFeaturesA[i].y + F.at<double>(0, 2);
        double B = F.at<double>(1, 0)*tFeaturesA[i].x + F.at<double>(1, 1)*tFeaturesA[i].y + F.at<double>(1, 2);
        double C = F.at<double>(2, 0)*tFeaturesA[i].x + F.at<double>(2, 1)*tFeaturesA[i].y + F.at<double>(2, 2);
        double dd = (A*tFeaturesB[i].x + B*tFeaturesB[i].y + C) / sqrt(A*A + B*B);
        tvdFDists.push_back(dd);
        //tTanSets.push_back(tLKTVector(1)/tLKTVector(0));
         */

    }
    double start = static_cast<double>(cvGetTickCount());
    std::vector<uchar> labels;
    cv::Mat mSamples(tAngleSets, false);

    cv::EM em_model;
    em_model.set("nclusters", 4);
    em_model.set("covMatType", EM::COV_MAT_SPHERICAL);
    em_model.train(mSamples, noArray(), labels, noArray());
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout <<"Cost "<< time/1000 << " ms" << endl;
    cv::Mat means = em_model.getMat("means");
    cout<< means<<endl;
    /*
    ComputeMaxBin(rotHist, HISTO_LENGTH, tvCommonVector);
    for (int k = 0; k < tvCommonVector.size(); ++k)
    {
        tOrient +=  tvdOrients[tvCommonVector[k]];

    }
    tOrient.normalize();

    tvdVectorx.clear();
    tvdVectory.clear();
    for (int l = 0; l < N; ++l)
    {
        tvdOrients[l] -= tOrient;

        tvdVectorx.push_back(tvdOrients[l][0]);
        tvdVectory.push_back(tvdOrients[l][1]);
    }


    std::vector<int> tBinNum;
    tBinNum.resize(21, 0);
    double tTanAve = 0;

    for (int j = 0; j < N; ++j)
    {
        double tOrientDiff = tOrient.dot(tvdOrients[j]);
        tOrients.push_back(tOrientDiff);

        //if(tOrientDiff < 0.90 && fabs(tvdFDists[j])>0.2)
        //    tvStatus[j] = 0;

        //tOrientAve += tOrientDiff/N;
        //if(abs(tDistAve - tvdDists[j]) > 0.3)
        //    tvStatus[j] = 0;
        //tAngleSets.push_back(atan(tTanSets[j]));
        //tTanAve = tAngleSets.back()/N;
    }


    std::ofstream outputFilex ("outputx.txt");
    outputFilex << " ";
    std::copy(tvdVectorx.begin(), tvdVectorx.end(), std::ostream_iterator<double>(outputFilex, " "));
    outputFilex << " " << std::endl;
    std::ofstream outputFiley ("outputxy.txt");
    outputFiley << " ";
    std::copy(tvdVectory.begin(), tvdVectory.end(), std::ostream_iterator<double>(outputFiley, " "));
    outputFiley << " " << std::endl;


    ReduceFeatures(tFeaturesA, tvStatus, &tBadPtsA);
    ReduceFeatures(tFeaturesB, tvStatus, &tBadPtsB);

    //mCam->Draw_Lines(mCurrentFrame.mColorImg, tFeaturesA, tFeaturesB, tBadPtsA, tBadPtsB);
    mCam->Show_Features(mCurrentFrame.mColorImg, tFeaturesA, rotHist);
    if (tFeaturesA.size() < 20)
    {
        mState = Lost;
        std::cout << "Too few features: " << tFeaturesA.size() << " after rarsac" << std::endl;
        //return;
    }
     */
    std::ofstream outputFilex ("angles.txt");
    outputFilex << " ";
    std::copy(tAngleSets.begin(), tAngleSets.end(), std::ostream_iterator<double>(outputFilex, " "));
    outputFilex << " " << std::endl;
    mCam->Show_Features(mCurrentFrame.mColorImg, tFeaturesA, labels);
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