#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_WARN("listening callback...");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);
    image_transport::Subscriber sub = image_transport.subscribe("camera/image_raw", 1, imageCallback);

    ros::spin();
}