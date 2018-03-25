//
// Created by buyi on 18-2-27.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include<ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <image_transport/image_transport.h>

#include "../../include/Camera.h"
#include "../../include/System.h"

using namespace std;


ros::Publisher Pose_pub;
geometry_msgs::PoseStamped Cam_Pose;

//tf::Transform DSDTM_tf;
Eigen::Vector3d Pose_Trans;
Eigen::Quaterniond Pose_Quat;
//tf::TransformBroadcaster *DSDTM_broadcaster;

int State = 0;

void Pub_CamPose(cv::Mat &pose);

class ImageGrabber
{
public:
    ImageGrabber(DSDTM::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    DSDTM::System* mpSLAM;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    //FLAGS_stderrthreshold = 0;
    FLAGS_log_dir = "/home/buyi/Personal/Slam_practice/DSDTM/log/TUM/longhouse";

    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc!=2)
    {
        cerr << endl << "Usage: rosrun DSDTM RGBD path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    DSDTM::System SLAM(argv[1], DSDTM::Camera_Model::RGB_PinHole, true);;

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    //!xtion
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
    Pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera/Tcw", 1);
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        //Pose_pub.publish(Cam_Pose);
        if(State==2)
        {
            static tf::TransformBroadcaster DSDTM_broadcaster;
            tf::Transform DSDTM_tf;

            DSDTM_tf.setOrigin(tf::Vector3(Pose_Trans[2], -Pose_Trans[0], -Pose_Trans[1]));
            DSDTM_tf.setRotation(tf::Quaternion(Pose_Quat.z(), -Pose_Quat.x(), -Pose_Quat.y(), Pose_Quat.w()));
            DSDTM_broadcaster.sendTransform(tf::StampedTransform(DSDTM_tf, ros::Time::now(), "/odom", "/camera_link"));
        }


        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop all threads
    //SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveCameraTrajectory("CameraTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rgb;
    cvtColor(cv_ptrRGB->image, rgb, COLOR_BGR2GRAY);
    Sophus::SE3 Pose = mpSLAM->TrackRGBD(rgb, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

    Pose_Trans = Pose.translation();
    Pose_Quat = Pose.unit_quaternion();

    State = mpSLAM->GetSystemState();
    /*
    Cam_Pose.header = cv_ptrRGB->header;

    Cam_Pose.pose.position.x = Pose.translation()[0];
    Cam_Pose.pose.position.y = Pose.translation()[1];
    Cam_Pose.pose.position.z = Pose.translation()[2];

    Cam_Pose.pose.orientation.w = Pose.unit_quaternion().w();
    Cam_Pose.pose.orientation.x = Pose.unit_quaternion().x();
    Cam_Pose.pose.orientation.y = Pose.unit_quaternion().y();
    Cam_Pose.pose.orientation.z = Pose.unit_quaternion().z();
     */
}