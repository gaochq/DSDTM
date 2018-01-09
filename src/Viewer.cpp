//
// Created by buyi on 18-1-9.
//

#include "Viewer.h"


namespace DSDTM
{

Viewer::Viewer(System *tSystem, Tracking *pTracking, MapDrawer *tMapDrawer):
        mSystem(tSystem), mMapDrawer(tMapDrawer), mbStopped(true), mbRequestStop(false)
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
    pangolin::Var<bool> menuStop("menu.Stop",true,true);
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

        mMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

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
        mMapDrawer->DrawCurrentCamera(Twc);

        if(menuShowKeyFrames)
            mMapDrawer->DrawKeyframes();

        if(menuShowPoints)
            mMapDrawer->DrawMapPoints();


        pangolin::FinishFrame();

        if(menuReset)
        {
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuStop = true;
            menuReset = false;
            menuFollowCamera = true;

            tbFollow = true;
            mbStopped = true;
            mbRequestStop = false;

            mSystem->Reset();
        }

        if(!menuStop && mbStopped)
        {
            RequestStart();
        }
        else if(menuStop && !mbStopped)
        {
            RequestStop();
        }

        if(Stop())
        {
            while(IsStopped() && menuStop)
            {
                usleep(3000);
            }
        }
    }
}

void Viewer::RequestStart()
{
    if(mbStopped)
        mbStopped = false;
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