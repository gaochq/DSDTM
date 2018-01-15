//
// Created by buyi on 17-11-24.
//

#include "Camera.h"
#include "Frame.h"
#include "Feature_detection.h"
#include "Feature.h"
#include "Tracking.h"
#include "Map.h"

#include <fstream>


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void Reject_FMat(std::vector<cv::Point2f> &_cur_Pts, std::vector<cv::Point2f> &_last_Pts);

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "./log";


    DSDTM::Map *mMap = new DSDTM::Map();
    DSDTM::Config::setParameterFile(argv[1]);
    DSDTM::CameraPtr camera(new DSDTM::Camera(DSDTM::Camera_Model::RGB_PinHole));
    DSDTM::Tracking tracking(camera, mMap);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages("/home/buyi/Datasets/EuRoC/mav0/cam0/data", "/home/buyi/Datasets/EuRoC/mav0/cam0/MH05.txt", vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;

    double start = static_cast<double>(cvGetTickCount());
    vector<cv::Point2f> next_keypoints;
    vector<cv::Point2f> prev_keypoints;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image
        color = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(color, last_color);

        tracking.Track_RGBDCam(last_color, color, 0);
        last_color.release();
    }
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout <<"Cost "<< time << " us" << endl;
    cout <<"Average "<< time/nImages << "us" << endl;

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
}
