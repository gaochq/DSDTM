//
// Created by buyi on 17-11-16.
//
#include "Camera.h"
#include "Frame.h"
#include "Keyframe.h"
#include "Feature_detection.h"
#include "Feature.h"
#include "Tracking.h"
#include "Map.h"

#include <fstream>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}


int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Usage: Test_Feature_detection Path_To_ParamFile" <<endl;
        return 0;
    }
    google::InstallFailureSignalHandler();

    DSDTM::Config::setParameterFile(argv[1]);
    DSDTM::Camera::CameraPtr camera(new DSDTM::Camera(argv[1],DSDTM::Camera_Model::RGB_PinHole));

    string Datasets_Dir = DSDTM::Config::Get<string>("dataset_dir");
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = Datasets_Dir + "/associations.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();

    cv::Mat ColorImage, Image_tmp, DepthIMage;

    std::vector<DSDTM::Frame*> mFrames;
    for (int i = 0; i < 2; ++i)
    {
        double start = static_cast<double>(cvGetTickCount());
        string Colorimg_path = Datasets_Dir + "/" + vstrImageFilenamesRGB[i];
        string Depthimg_path = Datasets_Dir + "/" + vstrImageFilenamesD[i];
        ColorImage = cv::imread(Colorimg_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
        DepthIMage = cv::imread(Depthimg_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(ColorImage, ColorImage);

        DSDTM::Frame* tFrame = new DSDTM::Frame(camera.get(), ColorImage, vTimestamps[i], DepthIMage);
        mFrames.push_back(tFrame);
    }

    DSDTM::Feature_detector *mFeature_detector = new DSDTM::Feature_detector();
    mFeature_detector->detect(mFrames[0], 20, false);

    std::vector<uchar> tvStatus;
    std::vector<float> tPyrLK_error;
    std::vector<cv::Point2f> tCur_Pts, tLast_Pts;

    //! LKT TRACKING
    mFrames[0]->Get_Features(tLast_Pts);
    cv::calcOpticalFlowPyrLK(mFrames[0]->mColorImg, mFrames[1]->mColorImg,
                             tLast_Pts, tCur_Pts, tvStatus, tPyrLK_error,
                             cv::Size(21, 21), 5);
    for (int i = 0; i < tCur_Pts.size(); ++i)
    {
        if (tvStatus[i] && !camera->IsInImage(tCur_Pts[i]))
        {
            tvStatus[i] = 0;
        }
    }
    DSDTM::Tracking::ReduceFeatures(tCur_Pts, tvStatus);
    DSDTM::Tracking::ReduceFeatures(tLast_Pts, tvStatus);
    tvStatus.clear();

    //! Reject relate to F
    cv::findFundamentalMat(tCur_Pts, tLast_Pts, cv::FM_RANSAC, 1.0, 0.99, tvStatus);
    DSDTM::Tracking::ReduceFeatures(tCur_Pts, tvStatus);
    DSDTM::Tracking::ReduceFeatures(tLast_Pts, tvStatus);

    mFrames[0]->ResetFrame();
    mFrames[0]->SetFeatures(tLast_Pts);
    mFrames[1]->SetFeatures(tCur_Pts);
    mFrames[0]->Set_Pose(Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

    DSDTM::KeyFrame *KFrame = new DSDTM::KeyFrame(*mFrames[0]);
    for (int i = 0; i <mFrames[0]->mvFeatures.size(); ++i)
    {
        float z = mFrames[0]->Get_FeatureDetph(mFrames[0]->mvFeatures[i]);
        if (z>0)
        {
            Eigen::Vector3d tPose = camera->Pixel2Camera(mFrames[0]->mvFeatures[i].mpx, z);
            DSDTM::MapPoint *tMPoint = new DSDTM::MapPoint(tPose, KFrame);

            mFrames[1]->Add_MapPoint(tMPoint, i);
        }
    }

    DSDTM::Optimizer::PoseSolver(*mFrames[1]);

    return  0;
}