#pragma once

#include <string>
#include <thread>
#include <chrono>
#include <math.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>


// Timing macros
#define MICROSECONDS(x) std::chrono::microseconds(x)
#define MILLISECONDS(x) std::chrono::milliseconds(x)
#define SECONDS(x) std::chrono::seconds(x)


#define RADIAN_TO_DEGREE M_PI/180   // Cte value. Defined before to don't need to calculate every call
#define TWO_PI           M_PI*2     // Cte value. Complete circle in radians

// class UavBase
// {
// public:
//     // Pure virtual functions. Needs to be override
//     virtual void gotoPositionAbsolute() = 0;
//     virtual void gotoPositionRelative(float x, float y, float z) = 0;
//     virtual void tookOff(float height) = 0;

//     virtual void rotateRadian(float angle) = 0;
//     void rotateDegree(float angle);

//     // Setters
//     void setCameraTopic(const std::string& topic);
//     void setOdometryTopic(const std::string& topic);
//     void setPoseTopic(const std::string& topic);

// protected:

//     // Topic Names
//     std::string sOdometryTopic      = "/pelican/odometry_sensor1/pose";
//     std::string sCameraTopic        = "/pelican/camera_monocular/image_raw";
//     std::string sGroundTruthTopic   = "/pelican/ground_truth/pose";
//     std::string sPoseTopic          = "/pelican/command/pose";

//     // UAV Publishers
//     ros::NodeHandle nodeHandler;
//     ros::Publisher posePublisher;

//     float _ERROR_POSE    = 0.2;      // Offset of the odometry position and target position
//     float _MAX_STEP_SIZE = 0.5;      // Max distance of a movement
//     float _Z_ERROR       = 0.14;     // Z axis error
// };



// class UavReal : public UavBase
// {
//     UavReal();

//     // Mavros
//     bool setupVehicle();
//     bool arm();
//     bool land();
//     bool setMode(std::string mode);
// };


class Uav
{
public:

    // Constructors
    Uav();
    Uav(const std::string& uavName);
    Uav(const std::string& uavName, const std::string& uavConfigPath);


    // Mavros
    bool setupVehicle();
    bool arm();
    bool land();
    bool setMode(const std::string mode);


    // Translation 
    void gotoPositionAbsolute(float x, float y, float z);
    void gotoPositionRelative(float x, float y, float z);
    void tookOff(float height);


    // Rotation
    void rotateRadian(float angle);
    void rotateDegree(float angle);


    // Setters
    void setCameraTopic(const std::string& topic);
    void setOdometryTopic(const std::string& topic);
    void setName(std::string uavName);
    void setTopicsNameBased();


    // Getters
    geometry_msgs::PoseConstPtr getOdometryPosition();
    geometry_msgs::PoseConstPtr getGroundTruthPosition();
    geometry_msgs::Quaternion getQuaternion();
    float getYaw();
    Eigen::Vector3f getPosition();


    // Helpers
    bool isAtPosition(float x, float y, float z);

private:

    std::string sUavName;

    // Topic Names
    std::string sOdometryTopic;
    std::string sCameraTopic;
    std::string sGroundTruthTopic;
    std::string sPoseTopic;

    // UAV Publishers
    ros::NodeHandle nodeHandler;
    ros::Publisher posePublisher;

    bool bAlreadyTookOff;
    bool bIsMoving;

    float _ERROR_POSE    = 0.2;      // Offset of the odometry position and target position
    float _MAX_STEP_SIZE = 0.5;      // Max distance of a movement
    float _Z_ERROR       = 0.05;     // Z axis error
};