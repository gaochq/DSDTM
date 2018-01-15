//
// Created by buyi on 17-10-16.
//

#ifndef DSDTM_CAMERA_H
#define DSDTM_CAMERA_H

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <memory>
#include "math.h"
#include <fstream>
#include <iterator>
#include <utility>
#include <thread>
#include <mutex>

#include "boost/bind.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/eigen.hpp>

#include <sophus/se3.h>

#include <pangolin/pangolin.h>

#include <glog/logging.h>

#include "tic_toc.h"
#include "Config.h"
#include "Feature.h"


namespace DSDTM
{

const Sophus::SE3 IdentitySE3 = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

enum Camera_Model
{
    RGB_PinHole,
    Mono_Pinhole,
    MEI
};
class Camera
{
public:
    Camera();
    Camera(const Camera_Model model);
    Camera(Camera_Model model, float f, float _fx, float _fy, float _cx, float _cy);
    ~Camera();

    //! Distortion correction for Feature in normalized plane
    Eigen::Vector3d LiftProjective(const Feature *feature, bool tUnDistort = true);

    //! Calculate the distortion vary in normalized
    void Distortion(const Eigen::Vector2d tPt, Eigen::Vector2d tvPt);

    //! Undistort Features
    cv::Point2f Undistort(const Feature tfeature);

    //! keypoint to vector
    Eigen::Vector2f Keypoint2Vector(const cv::KeyPoint &point);

    //! vector to keypoint
    cv::KeyPoint Vector2Keypoint(const Eigen::Vector2f &point);

    //! Camera to world
    Eigen::Vector3d Camera2World(const Sophus::SO3 &mT_cw, const Eigen::Vector3d &Point);

    //! World to camera
    Eigen::Vector3d World2Camera(const Sophus::SO3 &mT_cw, const Eigen::Vector3d &Point);

    //! Camera to pixel
    Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &Point);

    //! Pixel to Camera
    Eigen::Vector3d Pixel2Camera(const cv::Point2f &point, const float &depth);
    Eigen::Vector3d Pixel2Camera(const Eigen::Vector2d &point, const float &depth);

    //! Initalize the camera parameters
    void Init_CamParam();

    //! whether is the point in image
    bool IsInImage(const cv::Point2f _point, int tBoundary = 1, int tLevel = 0) const;

    //! Get the median value of the vector
    double GetMedian(const std::vector<double> tdVector) const;

    //! Calculate the variance using median absolute deviation
    void VarWithMAD(const std::vector<double> tErrors, std::vector<double> *tWeights);

    //! Draw features in image
    void Draw_Features(cv::Mat &_image, const Features _features);
    void Draw_Features(cv::Mat &_image, const std::vector<cv::Point2f> _features, std::vector<uchar> tStatus);

    //! Draw lines in image (mainly in the initialization)
    void Draw_Lines(cv::Mat _image, const Features _featuresA, const Features _featuresB);
    void Draw_Lines(cv::Mat _image, const std::vector<cv::Point2f> _featuresA, std::vector<cv::Point2f> _featuresB);

    //! Draw features in current image
    void Show_Features(const cv::Mat _image, const std::vector<cv::Point2f> _features, const std::vector<uchar> lables, uchar flag);

    //! Return Intrinsic matrix
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Return_Intrinsic() const
    {
        //Eigen::Matrix4d tIntrinsic;
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tIntrinsic;
        tIntrinsic << mfx, 0, mcx, mk1,
                      0, mfy, mcy, mk2,
                      0,   0,   1, mk3,
                    mp1, mp2,   0,   0;
        return tIntrinsic;
    }

    Eigen::Matrix3d Return_IntrinsicK()
    {
        Eigen::Matrix3d K;
        K  <<   mfx, 0, mcx,
                0, mfy, mcy,
                0,   0,   1;

        return K;
    }


public:

    Camera_Model    mCam_Model;
    float           mfx;
    float           mfy;
    float           mcx;
    float           mcy;
    float           mf;

    double          minv_fx;
    double          minv_fy;
    double          minv_cx;
    double          minv_cy;

    float           mk1;
    float           mk2;
    float           mp1;
    float           mp2;
    float           mk3;

    int             mheight;
    int             mwidth;

    int             mPyra_levels;

    cv::Mat         mInstrinsicMat;
    cv::Mat         mDistortionMat;

    std::vector<Eigen::Vector2d> mvGridBinPose;

};
typedef std::shared_ptr<Camera> CameraPtr;

//! Used for unaryExpr(), which can save a lot time than that in "for loop"
static int Eigenceil(double x)
{
    return ceil(x);
}

static int Eigenfloor(double x)
{
    return floor(x);
}

}//namespace DSDTM

#endif //DSDTM_CAMERA_H
