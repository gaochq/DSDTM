//
// Created by buyi on 17-10-17.
//

#include "Camera.h"
#include "Frame.h"
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
        cout << "Usage: test_Tracking Path_To_ParamFile" <<endl;
        return 0;
    }

    google::InitGoogleLogging(argv[0]);
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "./log/TUM/longhouse";

    DSDTM::System *tSystem = new DSDTM::System(argv[1],DSDTM::Camera_Model::RGB_PinHole, true);

    string Datasets_Dir = DSDTM::Config::Get<string>("dataset_dir");
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = Datasets_Dir + "/associations.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();
    cv::Mat ColorImage, Image_tmp, DepthIMage;
    double start = static_cast<double>(cvGetTickCount());
    for (int i = 0; i < nImages; ++i)
    {
        std::cout << i << std::endl;

        //! The single image cost almost 10ms on reading and clahe
        double start = static_cast<double>(cvGetTickCount());
        string Colorimg_path = Datasets_Dir + "/" + vstrImageFilenamesRGB[i];
        string Depthimg_path = Datasets_Dir + "/" + vstrImageFilenamesD[i];
        ColorImage = cv::imread(Colorimg_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
        DepthIMage = cv::imread(Depthimg_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);

//        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
//        clahe->apply(ColorImage, Image_tmp);

        Sophus::SE3 tPose = tSystem->TrackRGBD(ColorImage, DepthIMage, vTimestamps[i]);

        //Image_tmp.release();

        if(i > nImages - 10)
        {
            std::cout << vTimestamps[i] << "------" << tPose.translation().transpose() << std::endl;
        }
    }
    tSystem->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout <<"Cost "<< time << " us" << endl;
    cout <<"Average "<< time/nImages << "us" << endl;
    cout << tSystem->ReturnKeyFrameSize() << endl;

    return  0;
}