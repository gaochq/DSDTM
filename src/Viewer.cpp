//
// Created by buyi on 18-1-9.
//

#include "Viewer.h"


namespace DSDTM
{

Viewer::Viewer(System *tSystem, Tracking *pTracking, Map *tMap):
        mSystem(tSystem), mTracker(pTracking), mbStopped(false),
        mbRequestStop(false), mbPaused(true), mMap(tMap)
{
    float fps = Config::Get<int>("Camera.fps");
    if(fps < 1)
        fps = 30;
    mT = 1e3/fps;

    mImageWidth = Config::Get<int>("Camera.width");
    mImageHeight = Config::Get<int>("Camera.height");

    mViewerpointX = Config::Get<float>("Viewer.ViewpointX");
    mViewerpointY = Config::Get<float>("Viewer.ViewpointY");
    mViewerpointZ = Config::Get<float>("Viewer.ViewpointZ");
    mViewerpointF = Config::Get<float>("Viewer.ViewpointF");

    mfKeyFrameSize = Config::Get<float>("Viewer.KeyFrameSize");
    mfKeyFrameLineWidth = Config::Get<float>("Viewer.KeyFrameLineWidth");
    mfGraphLineWidth = Config::Get<float>("Viewer.GraphLineWidth");
    mfPointSize = Config::Get<float>("Viewer.PointSize");
    mfCameraSize = Config::Get<float>("Viewer.CameraSize");
    mfCameraLineWidth = Config::Get<float>("Viewer.CameraLineWidth");
}

void Viewer::DrawMapPoints()
{
    const std::vector<MapPoint*> &tvMapPoints = mMap->GetAllMapPoints();

    //! Draw all the MapPoints
    glPointSize(mfPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0; i < tvMapPoints.size(); ++i)
    {
        if(tvMapPoints[i]->Get_Pose().isZero() || tvMapPoints[i]->IsBad())
            continue;

        Eigen::Vector3f tPose = tvMapPoints[i]->Get_Pose().cast<float>();
        glVertex3f(tPose(0), tPose(1), tPose(2));
    }
    glEnd();

    //TODO Add the local MapPoints
}

void Viewer::DrawKeyframes()
{
    const float &w = mfKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const std::vector<KeyFrame*> tvKFrames = mMap->GetAllKeyFrames();
    int N = tvKFrames.size();

    /*
    for (size_t i = 0; i < N; ++i)
    {
        KeyFrame *tKFrame = tvKFrames[i];
        Sophus::SE3 tKfPose = tKFrame->Get_Pose().inverse();
        Eigen::Matrix<float, 4, 4, Eigen::ColMajor> tKfPoseMat = tKfPose.matrix().cast<float>();

        glPushMatrix();
        glMultMatrixf(tKfPoseMat.data());

        glLineWidth(mfKeyFrameLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);

        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(w, h, z);
        glVertex3f(-w, h, z);

        glVertex3f(w, -h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glEnd();

        glPopMatrix();
    }
    */

    //! Draw lines between two frames
    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (int j = 0; j < N; ++j)
    {
        Eigen::Vector3d tPose = tvKFrames[j]->Get_CameraCnt();
        glVertex3d(tPose(0), tPose(1), tPose(2));
    }
    glEnd();

    /*
    glLineWidth(mfKeyFrameLineWidth);
    glColor3f(0.0, 0.0, 1.0);
    Eigen::Vector3d tPose1 = Eigen::MatrixXd::Zero(3,1);
    Eigen::Vector3d tPose2 = Eigen::MatrixXd::Zero(3,1);
    glBegin(GL_LINES);
    for (int j = 0; j < N; ++j)
    {
        tPose2 = tvKFrames[j]->Get_CameraCnt();
        glVertex3d(tPose1(0), tPose1(1), tPose1(2));
        glVertex3d(tPose2(0), tPose2(1), tPose2(2));
        tPose1 = tPose2;
    }
    glEnd();
    */
}

void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mfCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();
    glMultMatrixd(Twc.m);

    glLineWidth(mfKeyFrameLineWidth);
    glColor3f(1.0f, 0.0f, 0.0f);

    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);

    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);

    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(w, h, z);
    glVertex3f(-w, h, z);

    glVertex3f(w, -h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);
    glEnd();

    glPopMatrix();
}

void Viewer::SetCurrentCameraPose(const Sophus::SE3 &Tcw)
{
    mLastCamPose = mCameraPose;
    mCameraPose = Tcw;
}

void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    //TODO test
    if(!mCameraPose.translation().isZero())
    {
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> tCameraPose(M.m);
        tCameraPose = mCameraPose.inverse().matrix();
    }
    else
        M.SetIdentity();
}

void Viewer::Run()
{
    //cv::namedWindow("DSDTM: Current Frame");
    pangolin::CreateWindowAndBind("DSDTM: Map Viewer",1024,768);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuPause("menu.Pause",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, mViewerpointF, mViewerpointF, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(mViewerpointX, mViewerpointY, mViewerpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                                                     .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool tbFollow = true;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        GetCurrentOpenGLCameraMatrix(Twc);

        //! choose the way to follow the Camera
        if(menuFollowCamera && tbFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !tbFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewerpointX, mViewerpointY, mViewerpointZ,
                                                               0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc);
            tbFollow = true;
        }
        else if(!menuFollowCamera && tbFollow)
        {
            tbFollow = false;
        }

        d_cam.Activate(s_cam);


        //！ Draw keyframs and mappoints
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        DrawCurrentCamera(Twc);

        if(menuShowKeyFrames)
            DrawKeyframes();

        if(menuShowPoints)
            DrawMapPoints();


        pangolin::FinishFrame();


        if(!menuPause)
        {
            mSystem->RequestStart();
        }
        else
        {
            mSystem->RequestPause();
            if(menuReset)
            {
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuPause = true;
                menuReset = false;
                menuFollowCamera = true;

                tbFollow = true;
                mbStopped = true;
                mbRequestStop = false;

                mTracker->Reset();
            }
        }

        if(Stop())
        {
            while(IsStopped())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
        }
    }
}

void Viewer::RequestStart()
{
    if(mbPaused)
        mbPaused = false;
}

void Viewer::RequestPause()
{
    if(!mbPaused)
        mbPaused = true;
}

void Viewer::RequestStop()
{
    if(!mbStopped)
        mbRequestStop = true;
}

bool Viewer::Stop()
{
    if(mbRequestStop)
    {
        mbStopped = true;
        mbRequestStop = false;
        return true;
    }

    return false;
}

bool Viewer::IsStopped()
{
    return mbStopped;
}

void Viewer::Release()
{
    mbStopped = true;
    mbRequestStop = false;
}

}// namesapce DSDTM