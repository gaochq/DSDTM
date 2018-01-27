//
// Created by buyi on 17-11-4.
//

#ifndef DSDTM_MAPPOINT_H
#define DSDTM_MAPPOINT_H

#include "Camera.h"
#include "Frame.h"
#include "Feature.h"
#include "Keyframe.h"
#include "Map.h"


namespace DSDTM
{

class KeyFrame;
class Frame;
class Map;

class MapPoint
{
public:
    MapPoint();
    MapPoint(Eigen::Vector3d& _pose, KeyFrame *_frame, Map *tMap);

    //! Set and Get the world position of mappoint
    void Set_Pose(Eigen::Vector3d tPose);
    Eigen::Vector3d Get_Pose() const;

    //! Add and delete Mappoint observation
    void Add_Observation(KeyFrame *tKFrame, size_t tfID);
    void Erase_Observation(KeyFrame *tKFrame);
    std::map<KeyFrame*, size_t> Get_Observations();

    //! Set mappoint bad
    void SetBadFlag();
    bool IsBad() const;

    //! Get observe and found times
    int Get_ObserveNums() const;
    int Get_FoundNums() const;

    //! Get the closest keyframe between cunrrent frame refer this MapPoint
    bool Get_ClosetObs(const Frame *tFrame, Feature *&tFeature, KeyFrame *&tKframe) const;

    //! Increase and Erase the match times with frames
    void IncreaseFound(int n = 1);
    void EraseFound(int n = 1);

    //! Increase the number of visible
    void IncreaseVisible(int n = 1);

    //! Get the index of feature in Keyframe oberse this MapPoint
    int Get_IndexInKeyFrame(KeyFrame *tKf);

    //! Update the normal vector direction and observe distance
    void UpdateNormalAndDepth();

    //! Return the Max and Min observe distance
    float Get_MaxObserveDistance();
    float Get_MinObserveDistance();

    //! Return the noraml vector
    Eigen::Vector3d Get_NormalVector();

    //! Get the ratio between found and visible
    float Get_FoundRatio();

protected:

public:
    unsigned long           mlID;
    static unsigned long    mlNextId;
    unsigned long           mLastSeenFrameId;
    unsigned long           mLastProjectedFrameId;
    unsigned long           mlLocalBAKFId;

    bool                    mbOutlier;

    double                  mdStaticWeight;

    float                   mfMaxDistance;
    float                   mfMinDistance;

    Eigen::Vector3d         mNoramlVector;

    unsigned long int   mFirstFrameId;

protected:

    Map     *mMap;

    Eigen::Vector3d     mPose;

    //! the frame first observe this point
    KeyFrame            *mRefKframe;

    //! Observation
    size_t                  mObsNum;
    std::map<KeyFrame*, size_t>         mObservations;

    //! Tracking counters
    int mnFound;
    int mnVisible;

};


} // namesapce DSDTM




#endif //DSDTM_MAPPOINT_H
