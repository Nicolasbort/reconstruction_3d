#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include "matcher.hpp"


bool takePicture_Pelican = false;
bool takePicture_Humming = false;

void takePictureCallback(std_msgs::Bool msg)
{
	takePicture_Pelican = msg.data;
	takePicture_Humming = msg.data;
}


int imWriteCount_Pelican = 0;
int imWriteCount_Humming = 0;


void imageCallback_Pelican(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

	if (takePicture_Pelican)
	{
		cv::imwrite("/home/nicolas/new_catkin/src/reconstruction_3d/images/pelican_" + std::to_string(imWriteCount_Pelican) + ".jpeg", img);
		ROS_INFO("Pelican saving image...\n");
		takePicture_Pelican = false;
		imWriteCount_Pelican++;
	}

	cv::resize(img, img, cv::Size(), 0.5, 0.5);

    cv::imshow("Pelican", img);
	

    cv::waitKey(20);
}


void imageCallback_Humming(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

	if (takePicture_Humming)
	{
		cv::imwrite("/home/nicolas/new_catkin/src/reconstruction_3d/images/humming_" + std::to_string(imWriteCount_Humming) + ".jpeg", img);
		ROS_INFO("Hummingbird saving image...\n");
		takePicture_Humming = false;
		imWriteCount_Humming++;
	}

	cv::resize(img, img, cv::Size(), 0.5, 0.5);

    cv::imshow("Humming", img);
	

    cv::waitKey(20);
}




int main(int argc, char **argv)
{
 	ros::init(argc, argv, "image_listener");

	ros::NodeHandle n;

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub_pelican = it.subscribe("/pelican/camera_monocular/image_raw", 1, imageCallback_Pelican);
	image_transport::Subscriber sub_humming = it.subscribe("/hummingbird/camera_monocular/image_raw", 1, imageCallback_Humming);

	ros::Subscriber sb = n.subscribe("/hydrone/camera/save_image", 10, takePictureCallback);
	
    ros::spin();
		
	cv::destroyWindow("view");
}