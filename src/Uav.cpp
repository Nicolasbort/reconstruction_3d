#include "Uav.hpp"


Uav::Uav() 
{
    ROS_INFO("Uav created");
    posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>(sPoseTopic, 10);
}

Uav::Uav(const std::string& uavConfigPath) {}



// Mavros
bool Uav::setupVehicle()
{
    if ( arm() && setMode("OFFBOARD") )
        return true;
    
    return false;
}

bool Uav::arm()
{
    mavros_msgs::CommandBool srv;

    srv.request.value = true;

    ros::ServiceClient client = nodeHandler.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    if (client.call(srv))
    {
        if (!srv.response.success)
        {
            ROS_ERROR("Failed to arm (Error Code: %d)", srv.response.result);
            return false;
        }

        ROS_INFO("Arm command sent");
        return true;
    }

    ROS_ERROR("Failed to arm");
    return false;
}

bool Uav::land()
{
    return true;
}

bool Uav::setMode(std::string mode)
{
    mavros_msgs::SetMode srv;

    srv.request.custom_mode = "OFFBOARD";

    ros::ServiceClient client = nodeHandler.serviceClient<mavros_msgs::SetMode>("/mavros/setmode");

    if (client.call(srv) && srv.response.mode_sent)
    {
        ROS_INFO("OFFBOARD mode set");
        return true;
    }

    ROS_ERROR("Failed to set mode");
    return false;
}


// Translation
void Uav::gotoPositionAbsolute(float x, float y, float z)
{
    ROS_INFO("gotoPositionAbsolute called...");

    geometry_msgs::PoseStamped newPosition;

    newPosition.pose.position.x = x;
    newPosition.pose.position.y = y;
    newPosition.pose.position.z = z;
    
    while (!isAtPosition(x, y, z))
    {  
        ROS_INFO("Moving...");
        posePublisher.publish(newPosition);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));    
    }  

    ROS_INFO("Arrived at [%.2f, %.2f, %.2f]", x, y, z);

}

void Uav::gotoPositionRelative(float x, float y, float z)
{
    ROS_INFO("gotoPositionRelative called...");

    geometry_msgs::PoseConstPtr sharedOdometry = this->getGroundTruthPosition();

    float newX   = sharedOdometry->position.x + x;
    float newY   = sharedOdometry->position.y + y;
    float newZ   = sharedOdometry->position.z + z; 

    this->gotoPositionAbsolute(newX, newY, newZ);
}

void Uav::tookOff(float height)
{
    ROS_INFO("Tooking off...");
    this->gotoPositionRelative(0, 0, height);
    sleep(1);

    // mavros_msgs::CommandTOL srv;

    // srv.request.altitude   = height;
    // srv.request.latitude   = 0;
    // srv.request.longitude  = 0;
    // srv.request.yaw        = 0;
    // srv.request.min_pitch  = 0;

    // ros::ServiceClient client = nodeHandler.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/tookoff");

    // if (client.call(srv))
    // {
    //     if (!srv.response.success)
    //     {
    //         ROS_ERROR("Failed to takeoff (Error Code: %d)", srv.response.result);
    //         return;
    //     }

    //     ROS_INFO("Takeoff command sent");
    //     return;
    // }

    // ROS_ERROR("Failed to tookoff");
}



// Rotation
void Uav::rotateRadian(float angle)
{
}

void Uav::rotateDegree(float angle)
{
    float angleRadians = angle * RADIAN_TO_DEGREE;

    rotateRadian(angleRadians);
}



// Setters
void Uav::setCameraTopic(const std::string& topic)
{
    sCameraTopic = topic;
}

void Uav::setOdometryTopic(const std::string& topic)
{
    sOdometryTopic = topic;
}



// Getters
geometry_msgs::PoseConstPtr Uav::getOdometryPosition()
{
    geometry_msgs::PoseConstPtr sharedPose;
    sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(this->sOdometryTopic, ros::Duration(10));
    return sharedPose;
}

geometry_msgs::PoseConstPtr Uav::getGroundTruthPosition()
{
    geometry_msgs::PoseConstPtr sharedPose;
    sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(this->sGroundTruthTopic, ros::Duration(10));
    return sharedPose;
}

float Uav::getYaw()
{

}


// Helpers
bool Uav::isAtPosition(float x, float y, float z)
{
    ROS_INFO("isAtPosition called...");

    geometry_msgs::PoseConstPtr sharedPose;
    sharedPose = this->getGroundTruthPosition();

    return fabs(sharedPose->position.x - x) < _ERROR_POSE &&
           fabs(sharedPose->position.y - y) < _ERROR_POSE &&
           fabs(sharedPose->position.z - z) < _ERROR_POSE - 0.05;
}