#include "matcher.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>



void reconstruct()
{
    ROS_FATAL("Reconstruction node started");

    bool start_reconstruction = false;
    std_msgs::Bool bool_msg;

    const int AMOUNT_FEATURES = 1500;

    Matcher matcher(AMOUNT_FEATURES, CIRCLE);


    String baseDir = "/home/nicolas/new_catkin/src/reconstruction_3d/images/";
    String prefix = "humming_";
    String ext = ".jpeg";
    vector<String> humming_images = {baseDir + prefix + "1" + ext, baseDir + prefix + "2" + ext,
                                      baseDir + prefix + "3" + ext, baseDir + prefix + "4" + ext,
                                      baseDir + prefix + "5" + ext, baseDir + prefix + "6" + ext};


    ROS_FATAL("Waiting message");
    // Wait till the reconstruction starts
    while (!start_reconstruction)
    {
        bool_msg  = *(ros::topic::waitForMessage<std_msgs::Bool>("/reconstruction/start"));
        start_reconstruction = bool_msg.data;
    }

    ROS_WARN("Starting reconstruction...");


    ROS_WARN("Loading images...");
    matcher.loadImages(humming_images);
    
    ROS_WARN("Resizing images...");
    matcher.resizeImages(0.7);

    ROS_WARN("Detecting features...");
    matcher.detectFeatures();

    ROS_WARN("Matching features...");
    matcher.match();
    
    ROS_WARN("Selecting good features...");
    matcher.selectGoodMatches();

    ROS_WARN("Showing matches...");
    matcher.showMatches();
}


int main(int argc, char **argv)
{
 	ros::init(argc, argv, "reconstruction_node");

    reconstruct();
	
    ros::spin();
		
	cv::destroyAllWindows();
}