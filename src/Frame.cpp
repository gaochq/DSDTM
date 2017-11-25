//
// Created by buyi on 17-10-16.
//

#include "Frame.h"
#include "Rarsac_base.h"


namespace DSDTM
{
unsigned long Frame::mlNextId = 0;

Frame::Frame()
{

}
// Copy contructor
Frame::Frame(Frame &frame):
        mCamera(frame.mCamera), mlId(frame.mlId), mdCloTimestamp(frame.mdCloTimestamp),
        mT_c2w(frame.mT_c2w), mColorImg(frame.mColorImg), mvImg_Pyr(frame.mvImg_Pyr),
        mDepthImg(frame.mDepthImg), mvFeatures(frame.mvFeatures), mPyra_levels(frame.mPyra_levels),
        mvGrid_probability(frame.mvGrid_probability), mvMapPoints(frame.mvMapPoints),
        mpObservation(frame.mpObservation)
{

}

// construct monocular frame
Frame::Frame(Camera* _cam, cv::Mat _colorIag, double _timestamp):
        mCamera(_cam), mColorImg(_colorIag), mdCloTimestamp(_timestamp)
{
    mlId = mlNextId++;
    InitFrame();
}

// construct rgbd frame
Frame::Frame(Camera *_cam, cv::Mat _colorImg, double _ctimestamp, cv::Mat _depthImg):
        mCamera(_cam), mColorImg(_colorImg), mdCloTimestamp(_ctimestamp),
        mDepthImg(_depthImg)
{
    mlId = mlNextId++;
    InitFrame();
}

Frame::~Frame()
{

}

void Frame::InitFrame()
{
    ResetFrame();
    mPyra_levels = Config::Get<int>("Camera.PyraLevels");
    mMin_Dist = Config::Get<int>("Camera.Min_dist");

    mvImg_Pyr.resize(mPyra_levels);
    ComputeImagePyramid(mColorImg, mvImg_Pyr);

    //cv::Mat tColorImg(mColorImg);
    //mColorImg.release();
    //cv::undistort(tColorImg, mColorImg, mCamera->mInstrinsicMat, mCamera->mDistortionMat);

    mImgMask = cv::Mat(mCamera->mheight, mCamera->mwidth, CV_8UC1, cv::Scalar(255));
}

void Frame::ResetFrame()
{
    mvFeatures.clear();
    mvGrid_probability.clear();
}

void Frame::ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr)
{
    Img_Pyr[0] = Image;
    for (int i = 1; i < mPyra_levels; ++i)
    {
        cv::pyrDown(Img_Pyr[i-1], Img_Pyr[i]);
    }
}

void Frame::Get_Features(std::vector<cv::Point2f> &_features)
{
    for (int i = 0; i < mvFeatures.size(); ++i)
    {
        _features.push_back(mvFeatures[i].mpx);
    }
}

void Frame::SetFeatures(const std::vector<cv::Point2f> &_features)
{
    for (int i = 0; i < _features.size(); ++i)
    {
        Add_Feature(Feature(this, _features[i], 0));
    }
}

void Frame::SetFeatures(const std::vector<cv::Point2f> &_features, std::vector<long int> tStatus,
                        std::vector<long int> tTrack_cnt)
{
    for (int i = 0; i < _features.size(); ++i)
    {
        Add_Feature(Feature(this, _features[i], 0, tStatus[i], tTrack_cnt[i]));
    }
}

float Frame::Get_FeatureDetph(const Feature feature)
{
    float p = mDepthImg.at<float>(feature.mpx.y, feature.mpx.x);
    return p;
}

void Frame::Add_MapPoint(size_t tFid, MapPoint *tPoint)
{
    mvMapPoints.push_back(tPoint);

    mpObservation.insert(std::pair<size_t, MapPoint* >(tFid, tPoint));
}

void Frame::Add_Observations(const KeyFrame &tKframe)
{
    std::map<long int, MapPoint*> tObservations = tKframe.Get_Observations();

    std::map<long int, MapPoint*>::iterator it;
    for (int i = 0; i < mvFeatures.size(); ++i)
    {
        it = tObservations.find(mvFeatures[i].mlId);
        if(it != tObservations.end())
            Add_MapPoint(i, it->second);
    }
}

void Frame::Add_Feature(Feature tfeature)
{
    mvFeatures.push_back(tfeature);

    cv::Point2f tUndistPt = mCamera->Undistort(tfeature);
    mvFeaturesUn.push_back(tUndistPt);
}

void Frame::Get_SceneDepth(double tMinDepth, double tMeanDepth)
{
    mMinDepth = std::numeric_limits<double>::max();
    std::vector<double> tDepth_vec;

    for (auto it = this->mvFeatures.begin(); it != this->mvFeatures.end(); it++)
    {
        const double z = Get_FeatureDetph(*it);
        if (z >= 0 && z < 10)
        {
            tDepth_vec.push_back(z);
            mMinDepth = fmin(mMinDepth, z);
        }
    }

    if (tDepth_vec.empty())
    {
        LOG(ERROR)<< "The scene depth is wrong in frame: %d" << this->mlId;
        return;
    }

    std::vector<double>::iterator iter = tDepth_vec.begin() + floor(tDepth_vec.size()/2);
    std::nth_element(tDepth_vec.begin(), iter, tDepth_vec.end());
    mMeanDepth = *iter;

    tMinDepth = mMinDepth;
    tMeanDepth = mMeanDepth;
}

bool Frame::Find_Observations(size_t tID)
{
    if(mpObservation.count(tID) > 0)
        return true;
    else
        return false;
}

Eigen::Vector3d Frame::UnProject(const cv::Point2f tPixel, const float d)
{
    Eigen::Vector3d tPoint = mCamera->Pixel2Camera(tPixel, d);
    return mT_c2w*tPoint;
}

void Frame::Set_Mask(std::vector<long int> &tlId, std::vector<long int> &tTrackCnt)
{
    tlId.clear();
    tTrackCnt.clear();
    mImgMask = cv::Mat(mCamera->mheight, mCamera->mwidth, CV_8UC1, cv::Scalar(255));

    std::sort(mvFeatures.begin(), mvFeatures.end());

    int j = 0;
    for (int i = 0; i < mvFeatures.size(); ++i)
    {
        if(mImgMask.at<uchar>(mvFeatures[i].mpx)==255)
        {
            mvFeatures[j++] = mvFeatures[i];
            tlId.push_back(mvFeatures[i].mlId);
            tTrackCnt.push_back(mvFeatures[i].mTrack_cnt);
            cv::circle(mImgMask, mvFeatures[i].mpx, mMin_Dist, 0, -1);
        }
    }
    mvFeatures.resize(j);
}

void Frame::Reset_Gridproba()
{
    mvGrid_probability.resize(100, 0.5);
}

}