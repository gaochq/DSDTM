//
// Created by buyi on 17-10-19.
//

#include "Initializer.h"
#include "Map.h"

namespace DSDTM
{

    Initializer::Initializer(Frame &frame, Camera* camera, Map *map):
            mCam(camera), mbInitSuccess(false), mMap(map)
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
            mFeature_detector->detect(&frame, 20, false);
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
            cv::waitKey(1);

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
                if (tvStatus[i] && !mCam->IsInImage(tCur_Pts[i]))
                {
                    tvStatus[i] = 0;
                }
            }
            ReduceFeatures(tCur_Pts, tvStatus);
            ReduceFeatures(tLast_Pts, tvStatus);
            tvStatus.clear();

            //! Reject relate to F
            cv::findFundamentalMat(tCur_Pts, tLast_Pts, cv::FM_RANSAC, 1.0, 0.99, tvStatus);
            ReduceFeatures(tCur_Pts, tvStatus);
            ReduceFeatures(tLast_Pts, tvStatus);


            //! The initial two frames have sanme features
            mReferFrame.ResetFrame();
            mReferFrame.SetFeatures(tLast_Pts);
            frame.SetFeatures(tCur_Pts);

            //! Show first frame
            cv::Mat tFeatureImg(frame.mColorImg);
            mCam->Draw_Lines(tFeatureImg, frame.mvFeatures, mReferFrame.mvFeatures);
            mCam->Draw_Features(tFeatureImg, frame.mvFeatures, cv::Scalar(0, 255, 0));
            cv::imshow("Feature_Detect", tFeatureImg);
            cv::waitKey(1);

            if (tCur_Pts.size()<50)
            {
                std::cout << "Too fewer features in Initializatier " << std::endl;
                return false;
            }
            else
            {
                KeyFrame *tKFrame = new KeyFrame(mReferFrame);
                mMap->AddKeyFrame(tKFrame);

                mReferFrame.Set_Pose(Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

                for (int i = 0; i < mReferFrame.mvFeatures.size(); ++i)
                {
                    mReferFrame.mvFeatures[i].mlId = i;
                    frame.mvFeatures[i].mlId = i;
                    tKFrame->mvFeatures[i].mlId = i;

                    float z = mReferFrame.Get_FeatureDetph(mReferFrame.mvFeatures[i]);
                    if (z>0)
                    {
                        Eigen::Vector3d tPose = mCam->Pixel2Camera(mReferFrame.mvFeatures[i].mpx, z);
                        MapPoint *tMPoint = new MapPoint(tPose, tKFrame);

                        tMPoint->Add_Observation(tKFrame, i);
                        tKFrame->Add_MapPoint(tMPoint);
                        tKFrame->Add_Observations(i, tMPoint);

                        mReferFrame.Add_MapPoint(i, tMPoint);
                        frame.Add_MapPoint(i, tMPoint);
                        mMap->AddMapPoint(tMPoint);

                    }
                }
                Optimizer::PoseSolver(frame);
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