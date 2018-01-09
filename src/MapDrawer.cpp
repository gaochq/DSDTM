//
// Created by buyi on 18-1-9.
//

#include "MapDrawer.h"


namespace DSDTM
{

MapDrawer::MapDrawer(Map *pMap):mMap(pMap)
{
    mfKeyFrameSize = Config::Get<float>("Viewer.KeyFrameSize");
    mfKeyFrameLineWidth = Config::Get<float>("Viewer.KeyFrameLineWidth");
    mfGraphLineWidth = Config::Get<float>("Viewer.GraphLineWidth");
    mfPointSize = Config::Get<float>("Viewer.PointSize");
    mfCameraSize = Config::Get<float>("Viewer.CameraSize");
    mfCameraLineWidth = Config::Get<float>("Viewer.CameraLineWidth");
}

void MapDrawer::DrawMapPoints()
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

void MapDrawer::DrawKeyframes()
{
    const float &w = mfKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const std::vector<KeyFrame*> tvKFrames = mMap->GetAllKeyFrames();

    for (size_t i = 0; i < tvKFrames.size(); ++i)
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

        //! Draw the line between two frames
        if( i >=1)
        {
            Sophus::SE3 tRelPose = tvKFrames[i-1]->Get_Pose()*tvKFrames[i]->Get_Pose().inverse();
            Eigen::Vector3f tRelTrans = tRelPose.inverse().translation().cast<float>();

            glLineWidth(mfKeyFrameLineWidth);
            glColor3f(0.0f, 0.0f, 1.0f);

            glBegin(GL_LINES);

            glVertex3f(0, 0, 0);
            glVertex3d(tRelTrans(0), tRelTrans(1), tRelTrans(2));

            glEnd();
        }

        glPopMatrix();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
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

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3 &Tcw)
{
    mLastCamPose = mCameraPose;
    mCameraPose = Tcw;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
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

}// namespace DSDTM