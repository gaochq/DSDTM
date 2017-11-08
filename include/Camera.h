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

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <sophus/se3.h>

#include "Config.h"
#include "Frame.h"
#include "Feature.h"


namespace DSDTM
{

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
    Camera(const std::string &Paramfile, const Camera_Model model);
    Camera(Camera_Model model, float _fx, float _fy, float _cx, float _cy);
    ~Camera();

    //! keypoint to vector
    Eigen::Vector2f Keypoint2Vector(const cv::KeyPoint &point);

    //! vector to keypoint
    cv::KeyPoint Vector2Keypoint(const Eigen::Vector2f &point);

    //! Camera to world
    Eigen::Vector3d Camera2World(const Sophus::SE3 &mT_cw, const Eigen::Vector3d &Point);

    //! World to camera
    Eigen::Vector3d World2Camera(const Sophus::SE3 &mT_cw, const Eigen::Vector3d &Point);

    //! Camera to pixel
    Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &Point);

    //! Pixel to Camera
    Eigen::Vector3d Pixel2Camera(const cv::Point2f &point, float &depth);

    //! Initalize the camera parameters
    void Init_CamParam(const std::string &Paramfile);

    //! whether is the point in image
    bool IsInImage(cv::Point2f _point);

    //! Draw features in image
    void Draw_Features(cv::Mat &_image, const Features _features, int _color);
    void Draw_Features(cv::Mat &_image, const std::vector<cv::Point2f> _features, cv::Scalar _color);

    //! Draw lines in image (mainly in the initialization)
    void Draw_Lines(cv::Mat &_image, const Features _featuresA, const Features _featuresB);

public:
    typedef std::shared_ptr<Camera> CameraPtr;

    Camera_Model    mCam_Model;
    float           mfx;
    float           mfy;
    float           mcx;
    float           mcy;

    float           mk1;
    float           mk2;
    float           mp1;
    float           mp2;
    float           mk3;

    int             mheight;
    int             mwidth;

    int             mPyra_levels;

};

}//namespace DSDTM

#endif //DSDTM_CAMERA_H
