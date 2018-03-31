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
typedef std::shared_ptr<Frame> FramePtr;

class KeyFrame
{
public:
    KeyFrame(const FramePtr _frame);
    ~KeyFrame();

    //! Add and Erase MapPoint
    void Add_MapPoint(MapPoint *tMpPoint, int tIdx);
    void Erase_MapPointMatch(MapPoint *tMpPoint);
    void Erase_MapPointMatch(int tIdx);

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

    //! Update conection between other KeyFrames
    void UpdateConnection();

    //! Update connection between the
    void AddConnection(KeyFrame *tKF, int tWeight);

    //! Update Covisibility Graph
    void UpdateCovGraph(KeyFrame *tKF, int tWeight);

    //! Return Covisibility Keyframes
    std::vector<std::pair<int, KeyFrame*>> GetCovKFrames();

    static bool CompareId(KeyFrame* pKF1, KeyFrame* pKF2)
    {
        return pKF1->mlId < pKF2->mlId;
    }

    std::vector<KeyFrame*> GetKeyFrameConnection(const int weight);

public:
    unsigned long int           mlId;
    static unsigned long int    mlNextId;
    const long              mRefId;
    const double            mTimeStamp;

    Features                mvFeatures;
    std::vector<MapPoint*>  mvMapPoints;

    std::map<long int, MapPoint*>   mpObservation;     //Feaure_id ---> Mappoint
    std::map<long int, size_t>      mpFFObservation;   //Feature_id ---> feature order in mvFeatures

    FramePtr                mFrame;
    CameraPtr               mCamera;

    cv::Mat                 mClolorImg;
    cv::Mat                 mDepthImg;
    std::vector<cv::Mat>    mvImg_Pyr;

    unsigned long           mlLocalBAKfId;
    unsigned long           mlFixedLocalBAKfId;

protected:

    int     mnVaildMps;     // the number of vaild Feature number

    Sophus::SE3             mT_c2w;
    Sophus::SO3             mR_w2c;
    Eigen::Vector3d         mOw;

    std::map<KeyFrame*, int>                    mConnectedKFs;
    std::vector<std::pair<int, KeyFrame*>>      mOrderedCovGraph;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
};

} //DSDTM



#endif //DSDTM_KEYFRAME_H
