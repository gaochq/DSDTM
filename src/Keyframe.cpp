//
// Created by buyi on 17-11-9.
//
#include "Keyframe.h"


namespace DSDTM
{
    KeyFrame::KeyFrame(Frame &tframe):
            mFrame(&tframe), mT_c2w(tframe.Get_Pose()), mvMapPoints(tframe.mvMapPoints),
            mvFeatures(tframe.mvFeatures)
    {
        std::for_each(tframe.mpObservation.begin(), tframe.mpObservation.end(), [&](std::pair<size_t , MapPoint*> tObs)
        {
           Add_Observations(tObs.first, tObs.second);
        });
    }

    KeyFrame::~KeyFrame()
    {

    }

    void KeyFrame::Add_MapPoint(MapPoint *tMPoint)
    {
        mvMapPoints.push_back(tMPoint);

    }

    void KeyFrame::Add_Observations(size_t tId, MapPoint *tMPoint)
    {
        mpFFObservation.insert(std::pair<long int, size_t>(mvFeatures[tId].mlId, tId));
        mpObservation.insert(std::pair<long int, MapPoint*>(tId, tMPoint));
        mvbOutliers.push_back(true);
    }




} //namesapce DSDTM