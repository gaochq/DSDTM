//
// Created by buyi on 17-11-9.
//
#include "Keyframe.h"


namespace DSDTM
{
long unsigned int KeyFrame::mlNextId=0;
KeyFrame::KeyFrame(Frame *tframe):
        mFrame(tframe), mvMapPoints(tframe->mvMapPoints),
        mvFeatures(tframe->mvFeatures), mnVaildMps(0), mCamera(tframe->mCamera),
        mClolorImg(tframe->mColorImg), mDepthImg(tframe->mDepthImg), mvImg_Pyr(tframe->mvImg_Pyr)
{
    mlId = mlNextId++;

    mvMapPoints.resize(mFrame->mvFeatures.size());
    Set_Pose(tframe->Get_Pose());

    std::for_each(tframe->mpObservation.begin(), tframe->mpObservation.end(), [&](std::pair<size_t , MapPoint*> tObs)
    {
        Add_Observations(tObs.first, tObs.second);
    });
}

KeyFrame::~KeyFrame()
{

}

void KeyFrame::Add_MapPoint(MapPoint *tMPoint, int tIdx)
{
    mvMapPoints[tIdx] = tMPoint;
    mnVaildMps++;
}

void KeyFrame::Add_Observations(size_t tId, MapPoint *tMPoint)
{
    mpFFObservation.insert(std::pair<long int, size_t>(mvFeatures[tId]->mlId, tId));
    mpObservation.insert(std::pair<long int, MapPoint*>(mvFeatures[tId]->mlId, tMPoint));
}

std::vector<MapPoint*> KeyFrame::GetMapPoints()
{
    return mvMapPoints;
}

void KeyFrame::Set_Pose(Sophus::SE3 tPose)
{
    mT_c2w = tPose;

    Sophus::SE3 tT_w2c = mT_c2w.inverse();
    mR_w2c = tT_w2c.so3();
    mOw = tT_w2c.translation();
}

Eigen::Vector2d KeyFrame::World2Pixel(const Eigen::Vector3d &Point)
{
    Eigen::Vector3d tPoint = mT_c2w*Point;

    return mCamera->Camera2Pixel(tPoint);
}

} //namesapce DSDTM