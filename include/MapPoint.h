//
// Created by buyi on 17-11-4.
//

#ifndef DSDTM_MAPPOINT_H
#define DSDTM_MAPPOINT_H

#include "Camera.h"
#include "Frame.h"
#include "Feature.h"
#include "Keyframe.h"


namespace DSDTM
{

class KeyFrame;
class Frame;

class MapPoint
{
public:
    MapPoint();
    MapPoint(Eigen::Vector3d& _pose, KeyFrame *_frame);

    //! Set and Get the world position of mappoint
    void Set_Pose(Eigen::Vector3d tPose);
    Eigen::Vector3d Get_Pose();

    //! Add Mappoint observation
    void Add_Observation(KeyFrame *tKFrame, size_t tfID);

    //! Set mappoint bad
    void SetBadFlag();
    bool IsBad() const;

    //! Get observe times
    int Get_ObserveNums() const;

    //! Get the closest keyframe between cunrrent frame refer this MapPoint
    bool Get_ClosetObs(const Frame *tFrame, Feature *&tFeature, KeyFrame *&tKframe);

    //! Increase the match times with frames
    void IncreaseFound(int n = 1);

protected:

public:
    unsigned long           mlID;
    static unsigned long    mlNextId;
    unsigned long           mLastSeenFrameId;
    unsigned long           mLastProjectedFrameId;

    bool                    mbOutlier;

protected:

    Eigen::Vector3d         mPose;

    //! the frame first observe this point
    unsigned long           mFirstFrame;

    //! Observation
    size_t                  mObsNum;
    std::map<KeyFrame*, size_t>         mObservations;

    //! Tracking counters
    int mnFound;
    int mnVisible;

};


} // namesapce DSDTM




#endif //DSDTM_MAPPOINT_H
