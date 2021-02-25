#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "Uav.hpp"

#define IS_SIMULATION true


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_planner_cpp");

    Uav uav;

    #if !IS_SIMULATION
        ROS_WARN("Isn't simulation\n");
        uav.setupVehicle();
    #else
        ROS_WARN("Is simulation\n");

        uav.tookOff(2.0);

        uav.gotoPositionAbsolute(10, 10, 2);

        uav.gotoPositionRelative(-3.0, 0, 0);

    #endif

    ros::spin();
}