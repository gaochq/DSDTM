//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_FEATURE_H
#define DSDTM_FEATURE_H

#include "Camera.h"

namespace DSDTM
{
class Frame;

struct Feature
{
    Frame*              mframe;
    cv::Point2f         mpx;
    int                 mlevel;
    long int            mlId;
    long int            mTrack_cnt;             // Succeed tracking times
    cv::Point2f         mUnpx;
    Eigen::Vector3d     mNormal;         //Unit-bearing vector of the feature. But now just for position of mappoint in last frame temporarily.
    Eigen::Vector3d     mPoint;

    Feature(Frame* _frame, const cv::Point2f& _px, int _level, long int tlId = -1, long int tTrack_cnt = 1):
            mframe(_frame),
            mpx(_px),
            mlevel(_level),
            mlId(tlId),
            mTrack_cnt(tTrack_cnt),
            mPoint(0.0, 0.0, 0.0)
    {

    }

    Feature(){};

    bool operator < (const Feature& _f1) const
    {
        return (_f1.mTrack_cnt < mTrack_cnt);
    }
};

typedef std::vector<Feature*> Features;
}// namespace DSDTM



#endif //DSDTM_FEATURE_H
