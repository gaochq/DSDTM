//
// Created by buyi on 18-1-5.
//
#include <iostream>
#include <string>
#include "System.h"

#include <fstream>

using namespace std;

bool tbFirstRead = false;
void LoadImages(const string &strImageFilename, vector<string> &vstrImageFilenamesRGB, vector<double> &vTimestamps,
                vector<Sophus::SE3> &tTransformSets)
{
    ifstream fAssociation;
    fAssociation.open(strImageFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);

            string sRGB;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);

            double t1, t2, t3;
            ss >> t1;
            ss >> t2;
            ss >> t3;
            Eigen::Vector3d Translation(t1, t2, t3);

            double q1,q2,q3,q4;
            ss >> q1;
            ss >> q2;
            ss >> q3;
            ss >> q4;
            Eigen::Quaterniond q(q4, q1, q2, q3);

            Sophus::SE3 Transform(q, Translation);
            tTransformSets.push_back(Transform);
        }
    }
}

void loadBlenderDepthmap(const std::string file_name, const DSDTM::CameraPtr cam, cv::Mat& img)
{
    std::ifstream file_stream(file_name.c_str());
    assert(file_stream.is_open());

    int tHeight = 480;
    int tWidth = 752;
    img = cv::Mat(tHeight, tWidth, CV_32FC1);
    float * img_ptr = img.ptr<float>();
    float depth;
    for(int y=0; y<tHeight; ++y)
    {
        for(int x=0; x<tWidth; ++x, ++img_ptr)
        {
            file_stream >> depth;
            Eigen::Vector3d point = cam->Pixel2Camera(cv::Point2f(x, y), 1.0);
            point.normalize();

            // blender:
            Eigen::Vector2d uv(point(0)/point(2), point(1)/point(2));
            *img_ptr = depth * sqrt(uv[0]*uv[0] + uv[1]*uv[1] + 1.0);

            // povray
            // *img_ptr = depth/100.0; // depth is in [cm], we want [m]

            if(file_stream.peek() == '\n' && x != tWidth-1 && y != tHeight-1)
                printf("WARNING: did not read the full depthmap!\n");
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
    FLAGS_log_dir = "./log/test_uzhsin";

    DSDTM::System *tSystem = new DSDTM::System(argv[1],DSDTM::Camera_Model::RGB_PinHole, true);
    DSDTM::CameraPtr tCamera = DSDTM::CameraPtr(new DSDTM::Camera(DSDTM::RGB_PinHole, 315.5, 315.5, 315.5, 376.0, 240.0));

    string Datasets_Dir = DSDTM::Config::Get<string>("dataset_dir");
    vector<string> vstrImageFilenamesRGB;
    vector<Sophus::SE3> mTransformSets;
    vector<double> vTimestamps;
    string strAssociationFilename = Datasets_Dir + "/trajectory.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vTimestamps, mTransformSets);

    int nImages = vstrImageFilenamesRGB.size();
    cv::Mat ColorImage, Image_tmp, DepthIMage;
    double start = static_cast<double>(cvGetTickCount());
    for (int i = 0; i < nImages; ++i)
    {
        std::cout <<"Frame " << i << "---->   ";

        //! The single image cost almost 10ms on reading and clahe
        double start = static_cast<double>(cvGetTickCount());
        std::string img_name(Datasets_Dir + "/img/" + vstrImageFilenamesRGB[i] + "_0.png");
        ColorImage = cv::imread(img_name, 0);
        DSDTM::TicToc tc;
        loadBlenderDepthmap(Datasets_Dir + "/depth/" + vstrImageFilenamesRGB[i] + "_0.depth", tCamera, DepthIMage);


        //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        //clahe->apply(ColorImage, Image_tmp);
        //double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
//        cout << time << "us" << endl;


        Sophus::SE3 tPose = tSystem->TrackRGBD(ColorImage, DepthIMage, vTimestamps[i]);
        std::cout <<"  "<< tc.toc() <<"  ";

        std::cout << "error: " << (tPose*mTransformSets[i].inverse()).translation().norm() <<"----"<< tPose.unit_quaternion().angularDistance(mTransformSets[i].unit_quaternion()) << std::endl;
    }
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout <<"Cost "<< time << " us" << endl;
    cout <<"Average "<< time/nImages << "us" << endl;

    return  0;
}