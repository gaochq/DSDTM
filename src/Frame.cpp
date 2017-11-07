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
            mvGrid_probability(frame.mvGrid_probability)
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
        mvFeatures.clear();
        for (int i = 0; i < _features.size(); ++i)
        {
            mvFeatures.push_back(Feature(this, _features[i], 0));
        }
    }

    float Frame::Get_FeatureDetph(const Feature feature)
    {
        //uchar *data = mDepthImg.data;
        //float z = *(data + static_cast<int>(feature.mpx.x*mDepthImg.step[0]
        //                                 + feature.mpx.y));
        float p = mDepthImg.at<float>(feature.mpx.y, feature.mpx.x);
        return p;
    }

    void Frame::Add_MapPoint(MapPoint *_Point)
    {
        mvMapPoints.push_back(_Point);
    }

    void Frame::Reset_Gridproba()
    {
        mvGrid_probability.resize(100, 0.5);
    }

}