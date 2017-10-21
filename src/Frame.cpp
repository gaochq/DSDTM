//
// Created by buyi on 17-10-16.
//

#include "Frame.h"
#include "Rarsac_base.h"


namespace DSDTM
{
    Frame::Frame()
    {

    }
    Frame::Frame(Frame &frame):
            mCamera(frame.mCamera), mId(frame.mId), mdCloTimestamp(frame.mdCloTimestamp),
            mdDepTimestamp(frame.mdDepTimestamp), mT_c2w(frame.mT_c2w), mColorImg(frame.mColorImg),
            mvImg_Pyr(frame.mvImg_Pyr), mDepthImg(frame.mDepthImg), mvFeatures(frame.mvFeatures),
            mPyra_levels(frame.mPyra_levels)
    {

    }

    Frame::Frame(Camera* _cam, cv::Mat _colorIag, double _timestamp):
            mCamera(_cam), mColorImg(_colorIag), mdCloTimestamp(_timestamp)
    {
        InitFrame();
    }

    Frame::Frame(Camera *_cam, cv::Mat _colorImg, double _ctimestamp,
                 cv::Mat _depthImg, double _dtimestamp):
            mCamera(_cam), mColorImg(_colorImg), mdCloTimestamp(_ctimestamp),
            mDepthImg(_depthImg), mdDepTimestamp(_dtimestamp)
    {
        InitFrame();
    }

    Frame::~Frame()
    {

    }

    void Frame::InitFrame()
    {
        mPyra_levels = Config::Get<int>("Camera.PyraLevels");
        mvImg_Pyr.resize(mPyra_levels);
        ComputeImagePyramid(mColorImg, mvImg_Pyr);
    }

    void Frame::ComputeImagePyramid(const cv::Mat Image, std::vector<cv::Mat> &Img_Pyr)
    {
        Img_Pyr[0] = Image;
        for (int i = 1; i < mPyra_levels; ++i)
        {
            cv::pyrDown(Img_Pyr[i-1], Img_Pyr[i]);
        }
    }

    void Frame::GetKeypoints(std::vector<cv::Point2f> &KeyPoints)
    {
        for (int i = 0; i < mvFeatures.size(); ++i)
        {
            KeyPoints.push_back(cv::Point2f(mvFeatures[i].mpx[0],
                                            mvFeatures[i].mpx[0]));
        }
    }

    void Frame::Reset_Gridproba()
    {
        mvGrid_probability.resize(100, 0.5);
    }
}