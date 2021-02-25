#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>


#define RADIAN_TO_DEGREE M_PI/180   // Constant value. Defined before to don't need to calculate every rotate call

class Uav
{
public:

    // Constructors
    Uav();
    Uav(const std::string& uavConfigPath);


    // Mavros
    bool setupVehicle();
    bool arm();
    bool land();
    bool setMode(std::string mode);


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


    // Getters
    geometry_msgs::PoseConstPtr getOdometryPosition();
    geometry_msgs::PoseConstPtr getGroundTruthPosition();
    float getYaw();


    // Helpers
    bool isAtPosition(float x, float y, float z);

private:

    // Topic Names
    std::string sOdometryTopic      = "/pelican/odometry_sensor1/pose";
    std::string sCameraTopic        = "/pelican/camera_monocular/image_raw";
    std::string sGroundTruthTopic   = "/pelican/ground_truth/pose";
    std::string sPoseTopic          = "/pelican/command/pose";

    // UAV Publishers
    ros::NodeHandle nodeHandler;
    ros::Publisher posePublisher;

    float _ERROR_POSE    = 0.2;      // Offset of the odometry position and target position
    float _MAX_STEP_SIZE = 0.5;      // Max distance of a movement
    float _Z_ERROR       = 0.14;     // Z axis error
};