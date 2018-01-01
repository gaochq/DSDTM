//
// Created by buyi on 17-12-25.
//

#include <iostream>
#include "Camera.h"
#include "Map.h"
#include "Feature_detection.h"
#include "Sprase_ImageAlign.h"

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

void loadBlenderDepthmap(const std::string file_name, const DSDTM::Camera::CameraPtr cam, cv::Mat& img)
{
    std::ifstream file_stream(file_name.c_str());
    assert(file_stream.is_open());
    img = cv::Mat(cam->mheight, cam->mwidth, CV_32FC1);
    float * img_ptr = img.ptr<float>();
    float depth;
    for(int y=0; y<cam->mheight; ++y)
    {
        for(int x=0; x<cam->mwidth; ++x, ++img_ptr)
        {
            file_stream >> depth;
            Eigen::Vector3d point = cam->Pixel2Camera(cv::Point2f(x, y), 1.0);
            point.normalize();

            // blender:
            Eigen::Vector2d uv(point(0)/point(2), point(1)/point(2));
            *img_ptr = depth * sqrt(uv[0]*uv[0] + uv[1]*uv[1] + 1.0);

            // povray
            // *img_ptr = depth/100.0; // depth is in [cm], we want [m]

            if(file_stream.peek() == '\n' && x != cam->mwidth-1 && y != cam->mheight-1)
                printf("WARNING: did not read the full depthmap!\n");
        }
    }
    cv::imwrite("depth.png", img);
}


int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Usage: test_SpraseImg_alignment Path_To_ParamFile" <<endl;
        return 0;
    }

    google::InitGoogleLogging(argv[0]);
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "./log";

    DSDTM::Config::setParameterFile(argv[1]);
    DSDTM::Camera::CameraPtr camera(new DSDTM::Camera(argv[1],DSDTM::Camera_Model::RGB_PinHole));

    string Datasets_Dir = DSDTM::Config::Get<string>("dataset_dir");
    vector<string> vstrImageFilenamesRGB;
    vector<Sophus::SE3> mTransformSets;
    vector<double> vTimestamps;
    string strAssociationFilename = Datasets_Dir + "/trajectory.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vTimestamps, mTransformSets);

    DSDTM::FramePtr frame_ref_;
    DSDTM::FramePtr frame_cur_;
    DSDTM::Feature_detector feature_detector;
    DSDTM::Sprase_ImgAlign mSpraseAlign(4, 0, 30);
    Sophus::SE3 T_prev_w, T_prevgt_w;

    list<double> mTranslationErrorSets;

    for (int i = 0; i < 30; ++i)
    {
        std::string img_name(Datasets_Dir + "/img/" + vstrImageFilenamesRGB[i] + "_0.png");
        cv::Mat img = cv::imread(img_name, 0);
        if(!img.data)
        {
            cout<< "Empty Image!" << endl;
            return 0;
        }

        Sophus::SE3 T_w_g = mTransformSets[i].inverse();

        if(i==0)
        {
            frame_ref_.reset(new DSDTM::Frame(camera.get(), img, vTimestamps[i]));
            frame_ref_->Set_Pose(T_w_g);

            cv::Mat depthmap;
            loadBlenderDepthmap(Datasets_Dir + "/depth/" + vstrImageFilenamesRGB[i] + "_0.depth", camera, depthmap);

            feature_detector.detect(frame_ref_.get(), 20);

            for_each(frame_ref_->mvFeatures.begin(), frame_ref_->mvFeatures.end(), [&](DSDTM::Feature &it)
            {
                float depth = depthmap.at<float>(it.mpx.y, it.mpx.x);
                Eigen::Vector3d pt_pos_cam = it.mNormal*depth;
                Eigen::Vector3d pt_pos_wd = frame_ref_->Get_Pose().inverse()*pt_pos_cam;
                it.mPoint = pt_pos_wd;
            });

            cout << "Added %d 3d pts to the reference frame" <<  frame_ref_->mvFeatures.size() << endl;
            T_prev_w = frame_ref_->Get_Pose();

            continue;
        }

        frame_cur_.reset(new DSDTM::Frame(camera.get(), img, vTimestamps[i]));
        frame_cur_->Set_Pose(T_prev_w);

        DSDTM::TicToc tc;
        mSpraseAlign.Run(frame_cur_, frame_ref_);
        //std::cout << tc.toc() << std::endl;

        Sophus::SE3 T_f_gt = frame_cur_->Get_Pose()*T_w_g.inverse();
        mTranslationErrorSets.push_back(T_f_gt.translation().norm());

        cout << i  << "---" << tc.toc() << "ms" <<"---"<< "Translation error: " << mTranslationErrorSets.back() << endl;

        T_prev_w = frame_cur_->Get_Pose();
    }
}