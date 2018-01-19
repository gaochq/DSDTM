//
// Created by buyi on 17-11-22.
//

#include "System.h"


namespace DSDTM
{

System::System(const std::string &Paramfile, const Camera_Model tSensor, const bool tbUseViewer):
        mSensor(tSensor), mbUseViewer(tbUseViewer), mbReseted(false), mbPaused(false)
{
    std::cout << "DSDTM" << std::endl;

    if(mSensor==Mono_Pinhole)
        cout << "Monocluar" <<std::endl;
    else if(mSensor==RGB_PinHole)
        std::cout << "RGBD" << std::endl;

    Config::setParameterFile(Paramfile);

    mCamera = CameraPtr(new Camera(mSensor));

    mMap = new Map();

    mMapDrawer = new MapDrawer(mMap);

    mLocalMapper = new LocalMapping(mMap);

    mTracker = new Tracking(mCamera, mMap, mLocalMapper, mMapDrawer);

    mViewer = new Viewer(this, mTracker, mMapDrawer);
    if(mbUseViewer)
        mtViewer = new std::thread(&Viewer::Run, mViewer);

    mTracker->SetViewer(mViewer);
}

Sophus::SE3 System::TrackRGBD(cv::Mat &tColorImg, cv::Mat &tDepthImg, const double &timestamp)
{
    if(mSensor!=RGB_PinHole)
    {
        LOG(ERROR)<< "Called wrong Tracking API!" << std::endl;
        std::cerr<< "Called wrong Tracking API!" << std::endl;
        exit(-1);
    }


    while(mbPaused==true)
    {
        //TODO Set the LocalMapper to sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return mTracker->Track_RGBDCam(tColorImg, tDepthImg, timestamp);
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    std::cout << "Saving camera trajectory!" <<std::endl;

    std::vector<KeyFrame*> tvKeyframs = mMap->GetAllKeyFrames();
    std::sort(tvKeyframs.begin(), tvKeyframs.end(), KeyFrame::CompareId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < tvKeyframs.size(); ++i)
    {
        KeyFrame *tkf = tvKeyframs[i];

        Eigen::Quaterniond q = tkf->Get_Pose().inverse().unit_quaternion();
        Eigen::Vector3f t = tkf->Get_CameraCnt().cast<float>();

        f << setprecision(6) << tkf->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << float(q.x()) << " " << float(q.y()) << " " << float(q.z()) << " " << float(q.w()) << endl;
    }
}

void System::Reset()
{
    mbReseted = true;
}

void System::RequestPause()
{
    mbPaused = true;
}

void System::RequestStart()
{
    mbPaused = false;
}


}