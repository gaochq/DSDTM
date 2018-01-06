//
// Created by buyi on 17-11-9.
//

#ifndef DSDTM_KEYFRAME_H
#define DSDTM_KEYFRAME_H

#include "Camera.h"
#include "Frame.h"
#include "MapPoint.h"

namespace DSDTM
{

class MapPoint;

class KeyFrame
{
public:
    KeyFrame(Frame *_frame);
    ~KeyFrame();

    //! Add MapPoint and the observation between mappoint and features
    void Add_MapPoint(MapPoint *tMPoint, int tIdx);
    void Add_Observations(size_t tId, MapPoint *tMPoint);

    //! Get the number of vaild mappoints
    int GetVaildMpNum() const { return mnVaildMps;}

    //! Return Observations
    std::map<long int, MapPoint*> Get_Observations() const { return mpObservation;};

    //! Return MapPoints
    std::vector<MapPoint*> GetMapPoints();

    //! Get and Set Keyframe pose
    Sophus::SE3 Get_Pose() const { return mT_c2w; }
    void Set_Pose(Sophus::SE3 tPose);

    //! Get camera center in world coordinate system
    Eigen::Vector3d Get_CameraCnt() const { return mOw; }
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d &Point);


public:
    unsigned long           mlId;
    static unsigned long    mlNextId;

    Features                mvFeatures;
    std::vector<MapPoint*>  mvMapPoints;

    std::map<long int, MapPoint*>   mpObservation;     //Feaure_id ---> Mappoint
    std::map<long int, size_t>      mpFFObservation;   //Feature_id ---> feature order in mvFeatures

    Frame                   *mFrame;

    cv::Mat                 mClolorImg;
    cv::Mat                 mDepthImg;
    std::vector<cv::Mat>     mvImg_Pyr;
protected:

    int     mnVaildMps;     // the number of vaild Feature number

    Sophus::SE3             mT_c2w;
    Sophus::SO3             mR_w2c;
    Eigen::Vector3d         mOw;

    CameraPtr               mCamera;
};

} //DSDTM



#endif //DSDTM_KEYFRAME_H
