#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "Uav.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_planner_cpp");

    Uav uav;

    // uav.setupVehicle();

    uav.tookOff(2.0);

    sleep(2);

    uav.gotoPositionAbsolute(10, 10, 2);

    // Test
    #if IS_SIMULATION
        ROS_WARN("Is simulation\n");
    #else
        ROS_WARN("Isn't simulation\n");
    #endif

    ros::spin();
}