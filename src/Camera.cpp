//
// Created by buyi on 17-10-16.
//

#include "Camera.h"


namespace DSDTM
{

Camera::Camera()
{

}

Camera::Camera(const Camera_Model model):
        mCam_Model(model)
{
    Init_CamParam();
}

Camera::Camera(Camera_Model model, float f, float _fx, float _fy, float _cx, float _cy):
        mCam_Model(model), mf(f), mfx(_fx), mfy(_fy), mcx(_cx), mcy(_cy)
{
}

Camera::~Camera()
{

}

void Camera::Init_CamParam()
{
    mf = Config::Get<float>("Camera.f");

    mfx = Config::Get<float>("Camera.fx");
    mfy = Config::Get<float>("Camera.fy");
    mcx = Config::Get<float>("Camera.cx");
    mcy = Config::Get<float>("Camera.cy");

    mk1 = Config::Get<float>("Camera.k1");
    mk2 = Config::Get<float>("Camera.k2");
    mp1 = Config::Get<float>("Camera.p1");
    mp2 = Config::Get<float>("Camera.p2");
    mk3 = Config::Get<float>("Camera.k3");

    mwidth = Config::Get<int>("Camera.width");
    mheight = Config::Get<int>("Camera.height");

    minv_fx = 1.0/mfx;
    minv_fy = 1.0/mfy;
    minv_cx = -mcx*minv_fx;
    minv_cy = -mcy*minv_fy;

    cv::Mat tInstrinsicMat = cv::Mat::eye(3,3,CV_32F);
    tInstrinsicMat.at<float>(0, 0) = mfx;
    tInstrinsicMat.at<float>(1, 1) = mfy;
    tInstrinsicMat.at<float>(0, 2) = mcx;
    tInstrinsicMat.at<float>(1, 2) = mcy;
    tInstrinsicMat.copyTo(mInstrinsicMat);

    cv::Mat tDistortMat(5, 1, CV_32F);
    tDistortMat.at<float>(0) = mk1;
    tDistortMat.at<float>(1) = mk2;
    tDistortMat.at<float>(2) = mp1;
    tDistortMat.at<float>(3) = mp2;
    tDistortMat.at<float>(4) = mk3;
    tDistortMat.copyTo(mDistortionMat);

    int tGridSize_Row = mheight/10;
    int tHalf_GridHeight = tGridSize_Row/2;
    int tGridSize_Col = mwidth/10;
    int tHalf_GridWidth = tGridSize_Col/2;
    for (int i = 0; i < 100; ++i)
    {
        mvGridBinPose.push_back(Eigen::Vector2d(tGridSize_Row*(i/10) + tHalf_GridHeight,
                                                tGridSize_Col*(i%10) + tHalf_GridWidth));
    }

    //std::cout<< mInstrinsicMat << std::endl << mDistortionMat <<std::endl;
}

Eigen::Vector3d Camera::LiftProjective(const Feature *feature, bool tUnDistort)
{
    Eigen::Vector3d P;
    double tmx_d, tmy_d, tmx_u, tmy_u;

    tmx_d = feature->mpx.x*minv_fx + minv_cx;
    tmy_d = feature->mpx.y*minv_fy + minv_cy;

    if(!tUnDistort)
    {
        P << tmx_d, tmy_d, 1;
    }

    //! Using Recursive distortion model
    else
    {
        int n = 8;
        Eigen::Vector2d tDisPt;
        Distortion(Eigen::Vector2d(tmx_d, tmy_d), tDisPt);

        tmx_u = tmx_d - tDisPt(0);
        tmy_u = tmy_d - tDisPt(1);

        for (int i = 0; i < n; ++i)
        {
            Distortion(Eigen::Vector2d(tmx_u, tmy_u), tDisPt);
            tmx_u = tmx_d - tDisPt(0);
            tmy_u = tmy_d - tDisPt(1);
        }

        P << tmx_u, tmy_u, 1;
    }

    return P;
}

void Camera::Distortion(const Eigen::Vector2d tPt, Eigen::Vector2d tvPt)
{
    double tPt_2x = tPt(0)*tPt(0);
    double tPt_2y = tPt(1)*tPt(1);
    double tpt_xy = tPt(0)*tPt(1);
    double tPt_r = tPt_2x + tPt_2y;

    double Radial_dist = mk1*tPt_r + mk2*tPt_r*tPt_r;

    tvPt(0) = tPt(0)*Radial_dist + 2*mp1*tpt_xy + mp2*(tPt_r + 2*tPt_2x);
    tvPt(1) = tPt(1)*Radial_dist + 2*mp2*tpt_xy + mp1*(tPt_r + 2*tPt_2y);
}

cv::Point2f Camera::Undistort(const Feature tfeature)
{
    double tmx_d, tmy_d, tmp2;

    tmx_d = tfeature.mpx.x*minv_fx + minv_cx;
    tmy_d = tfeature.mpx.y*minv_fy + minv_cy;
    tmp2 = tmx_d*tmx_d + tmy_d*+tmy_d;

    double distortion = 1.0 + tmp2*(mk1 + tmp2*mk2);

    return cv::Point2f(mfx*tmx_d*distortion + mcx,
                           mfy*tmy_d*distortion + mcy);
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

Eigen::Vector3d Camera::Camera2World(const Sophus::SO3 &mT_cw, const Eigen::Vector3d &Point)
{
    return mT_cw.inverse()*Point;
}

Eigen::Vector3d Camera::World2Camera(const Sophus::SO3 &mT_cw, const Eigen::Vector3d &Point)
{
    return mT_cw*Point;
}

Eigen::Vector2d Camera::Camera2Pixel(const Eigen::Vector3d &Point)
{
    return Eigen::Vector2d( mfx*Point[0]/Point[2] + mcx,
                            mfy*Point[1]/Point[2] + mcy);
}

Eigen::Vector3d Camera::Pixel2Camera(const cv::Point2f &point, const float &depth)
{
    return Eigen::Vector3d( depth*(point.x - mcx)/mfx,
                            depth*(point.y - mcy)/mfy,
                            depth);
}

Eigen::Vector3d Camera::Pixel2Camera(const Eigen::Vector2d &point, const float &depth)
{
    return Eigen::Vector3d( depth*(point(0) - mcx)/mfx,
                            depth*(point(1) - mcy)/mfy,
                            depth);
}

bool Camera::IsInImage(const cv::Point2f _point, int tBoundary, int tLevel) const
{
    if (cvRound(_point.x) >=tBoundary && cvRound(_point.x) < mwidth/(1<<tLevel) - tBoundary
        && cvRound(_point.y)>=tBoundary && cvRound(_point.y)< mheight/(1<<tLevel) - tBoundary)
        return true;
    return false;
}

void Camera::VarWithMAD(const std::vector<double> tErrors, std::vector<double> *tWeights)
{
    std::vector<double> tAbsErrors;
    double tmedian = 0;
    double tDegree = 1;

    for (int i = 0; i < tErrors.size(); ++i)
    {
        tAbsErrors.push_back(std::abs(tErrors[i] - tmedian));
    }

    std::sort(tAbsErrors.begin(), tAbsErrors.end());

    double tVariance = 1.4826*utils::GetMedian(tAbsErrors);

    for (int j = 0; j < tErrors.size(); ++j)
    {
        double tmp = tErrors[j]/tVariance;
        double tWeight = (tDegree + 1)/(tDegree + tmp*tmp);
        tWeights->push_back(tWeight);
    }
}

void Camera::Draw_Features(cv::Mat &_image, const Features _features)
{
    if (_image.channels() < 3)
        cv::cvtColor(_image, _image, CV_GRAY2BGR);

    for (int i = 0; i < _features.size(); ++i)
    {
        if(_features[i]->Mpt)
            cv::rectangle(_image,
                          cv::Point2f(_features[i]->mpx.x - 2, _features[i]->mpx.y - 2),
                          cv::Point2f(_features[i]->mpx.x + 2, _features[i]->mpx.y + 2),
                          cv::Scalar(0, 255, 0));
    };

    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", _image);
    cv::waitKey(1);
}

void Camera::Draw_Features(cv::Mat &_image, const std::vector<cv::Point2f> _features, std::vector<uchar> tStatus)
{
    if(_image.channels() < 3)
        cv::cvtColor(_image, _image, CV_GRAY2BGR);

    for (int i = 0; i < _features.size(); ++i)
    {
        if(tStatus[i]==0)
            cv::rectangle(_image,
                          cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                          cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                          cv::Scalar(0, 0, 255)); //r
        else
            cv::rectangle(_image,
                          cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                          cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                          cv::Scalar(0, 255, 0)); //g
    };

    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", _image);
    cv::waitKey(1);
}

void Camera::Draw_Lines(cv::Mat _image, const Features _featuresA, const Features _featuresB)
{
    if(_image.channels() < 3)
        cv::cvtColor(_image, _image, CV_GRAY2BGR);

    for (int i = 0; i < _featuresA.size(); ++i)
    {
        cv::line(_image, _featuresA[i]->mpx, _featuresB[i]->mpx, 1);
    }
}

void Camera::Draw_Lines(cv::Mat _image, const std::vector<cv::Point2f> _featuresA, std::vector<cv::Point2f> _featuresB)
{
    if(_image.channels() < 3)
        cv::cvtColor(_image, _image, CV_GRAY2BGR);

    for (int i = 0; i < _featuresA.size(); ++i)
    {
        cv::rectangle(_image,
                      cv::Point2f(_featuresA[i].x - 2, _featuresA[i].y - 2),
                      cv::Point2f(_featuresA[i].x + 2, _featuresA[i].y + 2),
                      cv::Scalar(0, 255, 0));
        cv::line(_image, _featuresA[i], _featuresB[i]+10*(_featuresA[i]- _featuresB[i]), cv::Scalar(255, 0, 0), 2);
    }

    cv::namedWindow("Optical_flow");
    cv::imshow("Optical_flow", _image);
    cv::waitKey(1);
}


void Camera::Show_Features(const cv::Mat _image, const std::vector<cv::Point2f> _features, const std::vector<uchar> lables, uchar flag)
{
    std::vector<cv::Scalar> tColors;
    tColors.push_back(cv::Scalar(0, 0, 255));       //bgr
    tColors.push_back(cv::Scalar(0, 255, 0));
    tColors.push_back(cv::Scalar(255, 0, 0));
    tColors.push_back(cv::Scalar(255, 0, 255));
    tColors.push_back(cv::Scalar(0, 255, 255));
    tColors.push_back(cv::Scalar(255, 0, 255));     //粉色

    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;
    cv::Mat Image_new = _image.clone();
    if(Image_new.channels() < 3)
        cv::cvtColor(Image_new, Image_new, CV_GRAY2BGR);
    for (int i = 0; i < _features.size(); ++i)
    {
        /*
        if(lables[i]==flag)
            cv::rectangle(Image_new,
                          cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                          cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                          tColors[0]);
        else
            cv::rectangle(Image_new,
                          cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                          cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                          tColors[1]);
        */

        if(lables[i]==0)
        {
            if(0==flag)
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[0]);

            else
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[1]);

            a++;
        }
        else if(lables[i]==1)
        {
            if(1==flag)
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[0]);

            else
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[1]);

            b++;
        }
        else if(lables[i]==2)
        {
            if(2==flag)
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[0]);

            else
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[1]);

            c++;
        }
        else
        {
            if(3==flag)
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[0]);

            else
                cv::rectangle(Image_new,
                              cv::Point2f(_features[i].x - 2, _features[i].y - 2),
                              cv::Point2f(_features[i].x + 2, _features[i].y + 2),
                              tColors[1]);

            d++;
        }
    }
    cv::namedWindow("Feature_Detect");
    cv::imshow("Feature_Detect", Image_new);
    cv::waitKey(1);
}

}// namespace DSDTM