//
// Created by buyi on 17-10-16.
//

#include "Camera.h"


namespace DSDTM
{
    Camera::Camera(const std::string &Paramfile, const Camera_Model model):
            mCam_Model(model)
    {
        Init_CamParam(Paramfile);
    }

    Camera::Camera(Camera_Model model, float _fx, float _fy, float _cx, float _cy):
            mCam_Model(model), mfx(_fx), mfy(_fy), mcx(_cx), mcy(_cy)
    {

    }

    Camera::~Camera()
    {

    }

    void Camera::Init_CamParam(const std::string &Paramfile)
    {
        mfx = Config::Get<float>("Camera.fx");
        mfy = Config::Get<float>("Camera.fy");
        mcx = Config::Get<float>("Camera.cx");
        mcy = Config::Get<float>("Camera.cy");

        mwidth = Config::Get<int>("Camera.width");
        mheight = Config::Get<int>("Camera.height");
    }

    Eigen::Vector2f Camera::Keypoint2Vector(const cv::KeyPoint &point)
    {
        return Eigen::Vector2f(point.pt.x, point.pt.y);
    }

    cv::KeyPoint Camera::Vector2Keypoint(const Eigen::Vector2f &point)
    {
        //! The third param is the diameter of the meaningful keypoint neighborhood
        return cv::KeyPoint(point[0], point[1], 1);
    }

    Eigen::Vector3d Camera::Camera2World(const Sophus::SE3 &mT_cw, const Eigen::Vector3d &Point)
    {
        return mT_cw.inverse()*Point;
    }

    Eigen::Vector3d Camera::World2Camera(const Sophus::SE3 &mT_cw, const Eigen::Vector3d &Point)
    {
        return mT_cw*Point;
    }

    Eigen::Vector2d Camera::Camera2Pixel(const Eigen::Vector3d &Point)
    {
        return Eigen::Vector2d( mfx*Point[0]/Point[2] + mcx,
                                mfy*Point[1]/Point[2] + mcy);
    }

    Eigen::Vector3d Camera::Pixel2Camera(const Eigen::Vector2d &point, double &depth)
    {
        return Eigen::Vector3d( depth*(point[0] - mcx)/mfx,
                                depth*(point[1] - mcy)/mfy,
                                depth);
    }

    bool Camera::IsInImage(cv::Point2f _point)
    {
        if (_point.x>=0 && _point.x<mheight && _point.y>=0 && _point.y<mwidth)
            return true;
        return false;
    }
}// namespace DSDTM