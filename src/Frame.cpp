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
    mvImg_Pyr.resize(mPyra_levels);

    ComputeImagePyramid(mColorImg, mvImg_Pyr);
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
        mvFeatures.push_back(Feature(this, _features[i], 0));
    }
}

void Frame::SetFeatures(const std::vector<cv::Point2f> &_features, std::vector<long int> tStatus)
{
    for (int i = 0; i < _features.size(); ++i)
    {
        mvFeatures.push_back(Feature(this, _features[i], 0, tStatus[i]));
    }
}

float Frame::Get_FeatureDetph(const Feature feature)
{
    float p = mDepthImg.at<float>(feature.mpx.y, feature.mpx.x);
    return p;
}

void Frame::Add_MapPoint(MapPoint *tPoint, size_t tFid)
{
    mvMapPoints.push_back(tPoint);

    mpObservation.insert(std::pair<size_t, MapPoint* >(tFid, tPoint));
}

void Frame::Add_Observations(const KeyFrame &tKframe)
{
    std::map<size_t, MapPoint*> tObservations = tKframe.Get_Observations();

    std::map<size_t, MapPoint*>::iterator it;
    for (int i = 0; i < mvFeatures.size(); ++i)
    {
        it = tObservations.find(mvFeatures[i].mlId);
        if(it != tObservations.end())
            Add_MapPoint(it->second, i);
    }
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

bool Frame::Find_Observations(long int tID)
{
    std::map<size_t , MapPoint*>::iterator it;

    it = mpObservation.find(tID);
    if(it!=mpObservation.end())
        return true;

    return false;
}

void Frame::Reset_Gridproba()
{
    mvGrid_probability.resize(100, 0.5);
}

}