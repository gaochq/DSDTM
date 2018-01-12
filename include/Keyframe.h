//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_KEYFRAME_H
#define DSDTM_KEYFRAME_H

#include "Camera.h"
#include "Frame.h"

namespace DSDTM
{

class MapPoint;
class Frame;

class KeyFrame
{
public:
    KeyFrame(Frame* tframe);
    ~KeyFrame();

    //! Add MapPoint and the observation between mappoint and features
    void Add_MapPoint(MapPoint *tMPoint);
    void Add_Observations(size_t tId, MapPoint *tMPoint);

    //! Return Observations
    std::map<long int, MapPoint*> Get_Observations() const { return mpObservation;};

    //! Return MapPoints
    std::vector<MapPoint*> GetMapPoints();

    //! Get and Set Keyframe pose
    Sophus::SE3 Get_Pose() { return mT_c2w; }
    void Set_Pose(Sophus::SE3 tPose) { mT_c2w = tPose; }


public:
    unsigned long           mlId;
    static unsigned long    mlNextId;

    Features                mvFeatures;
    std::vector<MapPoint*>  mvMapPoints;

    std::map<long int, MapPoint*>   mpObservation;     //Feaure_id ---> Mappoint
    std::map<long int, size_t>      mpFFObservation;   //Feature_id ---> feature order in mvFeatures

protected:
    Frame                   *mFrame;

    Sophus::SE3             mT_c2w;

};

} //DSDTM



#endif //DSDTM_KEYFRAME_H
