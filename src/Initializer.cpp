//
// Created by buyi on 17-10-19.
//

#include "Initializer.h"
#include "Map.h"

namespace DSDTM
{

Initializer::Initializer(FramePtr frame, CameraPtr camera, Map *map):
        mCam(camera), mbInitSuccess(false), mMap(map)
{

}

Initializer::~Initializer()
{

}

void Initializer::ReduceFeatures(Features &_features, std::vector<uchar> _status)
{
    for (int i = 0; i < _features.size(); ++i)
    {
        if(!_status[i])
            _features.erase(_features.begin() + i);
    }
}

void Initializer::ReduceFeatures(std::vector<cv::Point2f> &_points, std::vector<uchar> _status)
{
    for (int i = 0; i < _points.size(); ++i)
    {
        if(!_status[i])
            _points.erase(_points.begin() + i);
    }
}

bool Initializer::Init_RGBDCam(FramePtr frame)
{
    mFeature_detector = new Feature_detector();

    mFeature_detector->detect(frame.get(), 20, false);
    if (frame->Get_FeatureSize() < 50)
    {
        DLOG(ERROR) << "Too few features in Initializer" << std::endl;

        return false;
    }
    mReferFrame = frame;
    //mReferFrame->Set_Pose(Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
    //mReferFrame->Set_Pose(Sophus::SE3(Eigen::Quaterniond(0.00000, 0.82266, 0.21488, 0.00000), Eigen::Vector3d(0.11314, 0.11314, 2.00000)).inverse());
    mReferFrame->Set_Pose(Sophus::SE3(Eigen::Quaterniond(-0.0145, 0.0003, 0.8617, -0.5072), Eigen::Vector3d(-0.6832, 2.6909, 1.7373)).inverse());

    return true;
}

bool Initializer::Init_MonocularCam(Frame &lastFrame, Frame &currentFrame)
{
    return true;
}


}// namespace DSDTM