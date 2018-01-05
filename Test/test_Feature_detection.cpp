//
// Created by buyi on 17-10-17.
//

#include "Camera.h"
#include "Frame.h"
#include "Feature_detection.h"
#include "Feature.h"
#include "tic_toc.h"

#include <fstream>

using namespace std;

bool tbFirstRead = false;

void LoadImages(const string &strImageFilename, vector<string> &vstrImageFilenamesRGB, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strImageFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        //! read the first three lines of txt file
        if(!tbFirstRead)
        {
            getline(fAssociation, s);
            getline(fAssociation, s);
            getline(fAssociation, s);
            tbFirstRead = true;
        }
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
            //vstrImageFilenamesD.push_back(sD);

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

    google::InitGoogleLogging(argv[0]);
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "./log/FeatureDetection";

    DSDTM::Config::setParameterFile(argv[1]);
    DSDTM::CameraPtr camera(new DSDTM::Camera(argv[1],DSDTM::Camera_Model::RGB_PinHole));

    vector<double> dTimestamps;
    vector<string> dImageNames;
    string Datasets_Dir = DSDTM::Config::Get<string>("dataset_dir");
    string strImageFile = Datasets_Dir + "/rgb.txt";
    LoadImages(strImageFile, dImageNames, dTimestamps);

    int nImages = dImageNames.size();

    cv::Mat Image, Image_tmp;

    DSDTM::TicToc tc_sum;
    for (int i = 0; i < nImages; ++i)
    {
        //! The single image cost almost 10ms on reading and clahe
        DSDTM::TicToc tc_show;
        string img_path = Datasets_Dir + "/" + dImageNames[i];
        Image = cv::imread(img_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(Image, Image_tmp);

        //! The single image cost almost 4ms on extracing features
        DSDTM::FramePtr frame(new DSDTM::Frame(camera, Image_tmp, dTimestamps[i]));
        DSDTM::Features features;
        DSDTM::Feature_detector detector;

        if(i==100)
        {
            DSDTM::TicToc tc;
            detector.detect(frame.get(), 20);
            LOG(INFO) <<"Single image detection: " <<tc.toc() <<" ms"<<endl;
        }
        else
            detector.detect(frame.get(), 20);

        //cout << frame.get()->mvFeatures.size() << endl;


        cv::Mat Image_new = Image_tmp.clone();
        if(Image_new.channels() < 3)
            cv::cvtColor(Image_new, Image_new, CV_GRAY2BGR);
        for_each(frame.get()->mvFeatures.begin(), frame.get()->mvFeatures.end(), [&](DSDTM::Feature *feature)
        {
            cv::rectangle(Image_new,
                          cv::Point2f(feature->mpx.x - 2, feature->mpx.y - 2),
                          cv::Point2f(feature->mpx.x + 2, feature->mpx.y + 2),
                          cv::Scalar (0, 255, 0));
        });

        cv::namedWindow("Feature_Detect");
        cv::imshow("Feature_Detect", Image_new);
        cv::waitKey(1);

    }
    double time = tc_sum.toc();
    cout << time << "us" << endl;
    LOG(INFO) << "Average time: "<< time/nImages << " ms" << endl;
    return  0;
}