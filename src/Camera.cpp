//
// Created by buyi on 17-10-16.
//

#include "Camera.h"


namespace DSDTM
{
    Camera::Camera()
    {

    }

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

    Eigen::Vector3d Camera::Pixel2Camera(const cv::Point2f &point, float &depth)
    {
        return Eigen::Vector3d( depth*(point.x - mcx)/mfx,
                                depth*(point.y - mcy)/mfy,
                                depth);
    }

    bool Camera::IsInImage(cv::Point2f _point)
    {
        if (cvRound(_point.x) >=1 && cvRound(_point.x)<mwidth-1 && cvRound(_point.y)>=1 && cvRound(_point.y)< mheight-1)
            return true;
        return false;
    }

    void Camera::Draw_Features(cv::Mat &_image, const Features _features, cv::Scalar _color)
    {
        if(_image.channels() < 3)
            cv::cvtColor(_image, _image, CV_GRAY2BGR);

        std::for_each(_features.begin(), _features.end(), [&](Feature feature)
        {
            cv::rectangle(_image,
                          cv::Point2f(feature.mpx.x - 2, feature.mpx.y - 2),
                          cv::Point2f(feature.mpx.x + 2, feature.mpx.y + 2),
                          _color);
        });
    }

    void Camera::Draw_Features(cv::Mat &_image, const std::vector<cv::Point2f> _features, cv::Scalar _color)
    {
        if(_image.channels() < 3)
            cv::cvtColor(_image, _image, CV_GRAY2BGR);

        std::for_each(_features.begin(), _features.end(), [&](cv::Point2f feature)
        {
            cv::rectangle(_image,
                          cv::Point2f(feature.x - 2, feature.y - 2),
                          cv::Point2f(feature.x + 2, feature.y + 2),
                          _color);
        });
    }

    void Camera::Draw_Lines(cv::Mat &_image, const Features _featuresA, const Features _featuresB)
    {
        if(_image.channels() < 3)
            cv::cvtColor(_image, _image, CV_GRAY2BGR);

        for (int i = 0; i < _featuresA.size(); ++i)
        {
            cv::line(_image, _featuresA[i].mpx, _featuresB[i].mpx, 1);
        }
    }

    void Camera::Show_Features(const cv::Mat _image, const std::vector<cv::Point2f> _features, int _color)
    {
        cv::Mat Image_new = _image.clone();
        Draw_Features(Image_new, _features, _color);

        cv::namedWindow("Feature_Detect");
        cv::imshow("Feature_Detect", Image_new);
        cv::waitKey(1);
    }

    void Camera::Show_Features(const cv::Mat _image, const std::vector<cv::Point2f> _features1,
                               const std::vector<cv::Point2f> _features2, const std::vector<cv::Point2f> _features3)
    {
        cv::Mat Image_new = _image.clone();
        Draw_Features(Image_new, _features1, cv::Scalar(0, 0, 255));
        Draw_Features(Image_new, _features2, cv::Scalar(0, 255, 0));
        Draw_Features(Image_new, _features3, cv::Scalar(255, 0, 0));
        cv::namedWindow("Feature_Detect");
        cv::imshow("Feature_Detect", Image_new);
        cv::waitKey(1);
    }
}// namespace DSDTM