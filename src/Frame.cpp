//
// Created by buyi on 17-10-16.
//

#include "Frame.h"

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

    mvImg_Pyr.resize(mPyra_levels+1);
    ComputeImagePyramid(mColorImg, mvImg_Pyr);

    //! it takes almost 6ms to undistort the image, it takes too much time
    /*
    cv::Mat tColorImg(mColorImg);
    mColorImg.release();
    cv::undistort(tColorImg, mColorImg, mCamera->mInstrinsicMat, mCamera->mDistortionMat);
    */

    mImgMask = cv::Mat(mCamera->mheight, mCamera->mwidth, CV_8UC1, cv::Scalar(255));
    mDynamicMask = cv::Mat(mCamera->mheight, mCamera->mwidth, CV_8UC1, cv::Scalar(0));
}

void Frame::ResetFrame()
{
    mvFeatures.clear();
    mvGrid_probability.clear();
}

void Frame::ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr)
{
    Img_Pyr[0] = Image;
    for (int i = 1; i <= mPyra_levels; ++i)
    {
        Img_Pyr[i] = cv::Mat(Img_Pyr[i-1].rows/2, Img_Pyr[i-1].cols/2, CV_8U);
        halfSample(Img_Pyr[i-1], Img_Pyr[i]);
        //cv::pyrDown(Img_Pyr[i-1], Img_Pyr[i]);
    }
}

void Frame::halfSample(const cv::Mat &in, cv::Mat &out)
{
    assert( in.rows/2==out.rows && in.cols/2==out.cols);
    assert( in.type()==CV_8U && out.type()==CV_8U);

    const int stride = in.step.p[0];
    uint8_t* top = (uint8_t*) in.data;
    uint8_t* bottom = top + stride;
    uint8_t* end = top + stride*in.rows;
    const int out_width = out.cols;
    uint8_t* p = (uint8_t*) out.data;
    while (bottom < end)
    {
        for (int j=0; j<out_width; j++)
        {
            *p = static_cast<uint8_t>( (uint16_t (top[0]) + top[1] + bottom[0] + bottom[1])/4 );
            p++;
            top += 2;
            bottom += 2;
        }
        top += stride;
        bottom += stride;
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

void Frame::Set_Pose(Sophus::SE3 _pose)
{
    mT_c2w = _pose;

    Sophus::SE3 tT_w2c = mT_c2w.inverse();
    mR_w2c = tT_w2c.so3();
    mOw = tT_w2c.translation();
}

float Frame::Get_FeatureDetph(const Feature feature)
{
    float p = mDepthImg.at<float>(feature.mpx.y, feature.mpx.x);
    return p;
}

float Frame::Get_FeatureDetph(const cv::Point2f feature)
{
    float p = mDepthImg.at<float>(feature);
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
    tfeature.mNormal = mCamera->Pixel2Camera(tfeature.mpx, 1.0);
    tfeature.mNormal.normalize();
    mvFeatures.push_back(tfeature);
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

void Frame::UndistortFeatures()
{
    int N = mvFeatures.size();
    cv::Mat tMat(N, 2, CV_32F);
    std::vector<cv::Point2f> tSrc;
    std::vector<cv::Point2f> tmvFeaturesUn;           //Features undistorted

    for (int i = 0; i < N; ++i)
    {
        tSrc.push_back(mvFeatures[i].mpx);
    }

    //! the InputArray in undistortPoints should be 2 channels
    cv::undistortPoints(tSrc, tmvFeaturesUn, mCamera->mInstrinsicMat,
                        mCamera->mDistortionMat, cv::noArray(), mCamera->mInstrinsicMat);

    for (int j = 0; j < N; ++j)
    {
        mvFeatures[j].mUnpx = tmvFeaturesUn[j];

        mvFeatures[j].mNormal = mCamera->LiftProjective(mvFeatures[j], false);
    }
}

Eigen::Vector3d Frame::UnProject(const cv::Point2f tPixel, const float d)
{
    Eigen::Vector3d tPoint = mCamera->Pixel2Camera(tPixel, d);

    return mT_c2w.inverse()*tPoint;
}

void Frame::Set_Mask(std::vector<long int> &tlId, std::vector<long int> &tTrackCnt,
                     std::vector<cv::Point2f> tBadPts)
{
    tlId.clear();
    tTrackCnt.clear();
    mImgMask = cv::Mat(mCamera->mheight, mCamera->mwidth, CV_8UC1, cv::Scalar(255));

    //! Sort features refer to tracking times
    std::sort(mvFeatures.begin(), mvFeatures.end());

    mImgMask = mImgMask - mDynamicMask;
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

    for (int k = 0; k < tBadPts.size(); ++k)
    {
        cv::circle(mImgMask, tBadPts[k], mMin_Dist, 0, -1);
    }
    mvFeatures.resize(j);
}

bool Frame::isVisible(const Eigen::Vector3d tPose, int tBoundary) const
{
    Eigen::Vector3d tPoseCam = mT_c2w*tPose;
    if(tPoseCam(3) < 0.0)
        return false;

    Eigen::Vector2d px = mCamera->Camera2Pixel(tPoseCam);
    if(mCamera->IsInImage(cv::Point2f(px(0), px(1)), tBoundary))
        return true;

    return false;
}

void Frame::Reset_Gridproba()
{
    mvGrid_probability.resize(100, 0.5);
}

Eigen::Vector2d Frame::World2Pixel(const Eigen::Vector3d &Point)
{
    Eigen::Vector3d tPoint = mT_c2w*Point;

    return mCamera->Camera2Pixel(tPoint);
}

}