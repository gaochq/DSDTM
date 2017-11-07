//
// Created by buyi on 17-10-19.
//

#include "Initializer.h"

namespace DSDTM
{

    Initializer::Initializer(Frame &frame, Camera* camera):
            mCam(camera), mbInitSuccess(false)
    {

    }

    Initializer::~Initializer()
    {

    }

    void Initializer::ReduceFeatures(Features &_features, std::vector<uchar> _status)
    {
        for (int i = 0; i < _features.size(); ++i)
        {
            if(!_status[i])
                _features.erase(_features.begin() + i);
        }
    }

    void Initializer::ReduceFeatures(std::vector<cv::Point2f> &_points, std::vector<uchar> _status)
    {
        for (int i = 0; i < _points.size(); ++i)
        {
            if(!_status[i])
                _points.erase(_points.begin() + i);
        }
    }

    bool Initializer::Init_RGBDCam(Frame &frame)
    {
        mFeature_detector = new Feature_detector();

        if(!mbInitSuccess)
        {
            mFeature_detector->detect(&frame, 20);
            if (frame.Get_FeatureSize() < 50)
            {
                std::cout << "Too few features in Initalizr" << std::endl;

                return false;
            }

            mReferFrame = Frame(frame);

            //! Show first frame
            cv::namedWindow("Feature_Detect");
            cv::Mat tFeatureImg(mReferFrame.mColorImg);
            mCam->Draw_Features(tFeatureImg, mReferFrame.mvFeatures, 0);
            cv::imshow("Feature_Detect", tFeatureImg);

            mbInitSuccess = true;
            return false;
        }
        else
        {
            std::vector<uchar> tvStatus;
            std::vector<float> tPyrLK_error;
            std::vector<cv::Point2f> tCur_Pts, tLast_Pts;

            //! LKT TRACKING
            mReferFrame.Get_Features(tLast_Pts);
            cv::calcOpticalFlowPyrLK(mReferFrame.mColorImg, frame.mColorImg,
                                     tLast_Pts, tCur_Pts, tvStatus, tPyrLK_error,
                                     cv::Size(21, 21), 5);
            for (int i = 0; i < tCur_Pts.size(); ++i)
            {
                if (mCam->IsInImage(tCur_Pts[i]))
                    continue;
                else
                {
                    tCur_Pts.erase(tCur_Pts.begin() + i);
                    tLast_Pts.erase(tLast_Pts.begin() + i);
                    i--;
                }

            }
            ReduceFeatures(tCur_Pts, tvStatus);
            ReduceFeatures(tLast_Pts, tvStatus);
            tvStatus.clear();

            //! Reject relate to F
            cv::findFundamentalMat(tCur_Pts, tLast_Pts, cv::FM_RANSAC, 3, 0.99, tvStatus);
            ReduceFeatures(tCur_Pts, tvStatus);
            ReduceFeatures(tLast_Pts, tvStatus);
            mReferFrame.SetFeatures(tLast_Pts);
            frame.SetFeatures(tCur_Pts);

            //! Show first frame
            cv::namedWindow("Feature_Detect");
            cv::Mat tFeatureImg(frame.mColorImg);
            mCam->Draw_Lines(tFeatureImg, frame.mvFeatures, mReferFrame.mvFeatures);
            cv::imshow("Feature_Detect", tFeatureImg);

            if (tCur_Pts.size()<50)
            {
                std::cout << "Too fewer features in Initializatier " << std::endl;
                return false;
            }
            else
            {
                mReferFrame.Set_Pose(Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

                for (int i = 0; i < mReferFrame.mvFeatures.size(); ++i)
                {
                    float z = mReferFrame.Get_FeatureDetph(mReferFrame.mvFeatures[i]);
                    float p = mReferFrame.mDepthImg.at<float>(mReferFrame.mvFeatures[i].mpx.x, mReferFrame.mvFeatures[i].mpx.y);
                    Eigen::Vector3d tPose = mCam->Pixel2Camera(mReferFrame.mvFeatures[i].mpx, z);
                    MapPoint *tNewPoint = new MapPoint(tPose, &frame);
                    mReferFrame.Add_MapPoint(tNewPoint);
                }
                std::cout << "Initalize RGBD Camera successfully ! " << std::endl;
                return true;
            }
        }


    }

    bool Initializer::Init_MonocularCam(Frame &lastFrame, Frame &currentFrame)
    {
        return true;
    }


}// namespace DSDTM