#include "Uav.hpp"


Uav::Uav() 
{
    // Default UAV
    setName("pelican");

    ROS_INFO("Uav created name %s", sUavName.c_str());

    posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>(sPoseTopic, 10);

    bAlreadyTookOff = false;
    bIsMoving = false;
}


Uav::Uav(const std::string& uavName)
{
    setName(uavName);

    ROS_INFO("Uav created name %s", sUavName.c_str());

    posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>(sPoseTopic, 10);

    bAlreadyTookOff = false;
    bIsMoving = false;
}

// Not implemented
Uav::Uav(const std::string& uavName, const std::string& uavConfigPath) 
{
    ROS_INFO("Uav created using configPath");
    bAlreadyTookOff = false;
    bIsMoving = false;
}


//
// Mavros
//
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

bool Uav::setMode(const std::string mode)
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


//
// Translation
//
void Uav::gotoPositionAbsolute(float x, float y, float z)
{
    // ROS_INFO("gotoPositionAbsolute called...");

    geometry_msgs::PoseStamped newPosition;

    newPosition.pose.position.x = x;
    newPosition.pose.position.y = y;
    newPosition.pose.position.z = z;

    // Orientation keeps the same
    newPosition.pose.orientation = this->getQuaternion();
    
    while (!isAtPosition(x, y, z))
    {  
        // ROS_INFO("Moving...");
        posePublisher.publish(newPosition);
        std::this_thread::sleep_for(MILLISECONDS(800));
    }  

    // ROS_INFO("Arrived at [%.2f, %.2f, %.2f]", x, y, z);

}

void Uav::gotoPositionRelative(float x, float y, float z)
{
    // ROS_INFO("gotoPositionRelative called...");

    geometry_msgs::PoseConstPtr sharedOdometry = this->getGroundTruthPosition();

    float newX   = sharedOdometry->position.x + x;
    float newY   = sharedOdometry->position.y + y;
    float newZ   = sharedOdometry->position.z + z; 

    this->gotoPositionAbsolute(newX, newY, newZ);
}

void Uav::tookOff(float height)
{
    if ( this->bAlreadyTookOff){
        ROS_INFO("UAV Already took off...");
    }
    else{
        ROS_INFO("Tooking off...");
        this->bAlreadyTookOff = true;
        this->gotoPositionRelative(0, 0, height);
        std::this_thread::sleep_for(SECONDS(2));
    }
    

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


//
// Rotation
//
void Uav::rotateRadian(float angle)
{
    geometry_msgs::PoseConstPtr sharedPose = this->getGroundTruthPosition();

    float w = sharedPose->orientation.w;
    float x = sharedPose->orientation.x;
    float y = sharedPose->orientation.y;
    float z = sharedPose->orientation.z;

    Eigen::Quaternionf currentQuaternion(w, x, y, z);
    Eigen::Quaternionf rotationQuaternion( cos(angle/2.0), 0, 0, sin(angle/2.0) );
    Eigen::Quaternionf newAngleQuaternion = currentQuaternion * rotationQuaternion;

    Eigen::Vector4f quaternionCoeffs = newAngleQuaternion.coeffs();

    geometry_msgs::PoseStamped newPose;

    newPose.pose.position = sharedPose->position;

    newPose.pose.orientation.x = quaternionCoeffs(0);
    newPose.pose.orientation.y = quaternionCoeffs(1);
    newPose.pose.orientation.z = quaternionCoeffs(2);
    newPose.pose.orientation.w = quaternionCoeffs(3);

    ROS_INFO("Rotating %.2f radians", newAngleQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)(2));
    posePublisher.publish(newPose);

    std::this_thread::sleep_for(SECONDS(5));
}

void Uav::rotateDegree(float angle)
{
    float angleRadians = angle * RADIAN_TO_DEGREE;

    rotateRadian(angleRadians);
}


//
// Setters
//
void Uav::setCameraTopic(const std::string& topic)
{
    sCameraTopic = topic;
}

void Uav::setOdometryTopic(const std::string& topic)
{
    sOdometryTopic = topic;
}

void Uav::setName(std::string uavName)
{
    sUavName = uavName;

    setTopicsNameBased();
}

void Uav::setTopicsNameBased()
{
    if (sUavName.empty()){
        ROS_WARN("Uav name empty! Can't set topic names");
        return;
    }

    sOdometryTopic     = "/" + sUavName + "/odometry_sensor1/pose";
    sCameraTopic       = "/" + sUavName + "/camera_monocular/image_raw";
    sGroundTruthTopic  = "/" + sUavName + "/ground_truth/pose";
    sPoseTopic         = "/" + sUavName + "/command/pose";
}


//
// Getters
//
geometry_msgs::PoseConstPtr Uav::getOdometryPosition()
{
    geometry_msgs::PoseConstPtr sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(this->sOdometryTopic, ros::Duration(10));;
    return sharedPose;
}

geometry_msgs::PoseConstPtr Uav::getGroundTruthPosition()
{
    geometry_msgs::PoseConstPtr sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>(this->sGroundTruthTopic, ros::Duration(10));
    return sharedPose;
}

geometry_msgs::Quaternion Uav::getQuaternion()
{
    geometry_msgs::PoseConstPtr sharedPose = getGroundTruthPosition();
    return sharedPose->orientation;
}

float Uav::getYaw()
{
    geometry_msgs::PoseConstPtr sharedPose = this->getGroundTruthPosition();

    float w = sharedPose->orientation.w;
    float x = sharedPose->orientation.x;
    float y = sharedPose->orientation.y;
    float z = sharedPose->orientation.z;

    Eigen::Quaternionf quaternion(w, x, y, z);

    Eigen::Vector3f euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    return euler(2);
}

Eigen::Vector3f Uav::getPosition()
{
   geometry_msgs::PoseConstPtr sharedPose = this->getGroundTruthPosition();

   Eigen::Vector3f position;

   position(0) = sharedPose->position.x; 
   position(1) = sharedPose->position.y; 
   position(2) = sharedPose->position.z;

   return position;
}


//
// Helpers
//
bool Uav::isAtPosition(float x, float y, float z)
{
    // ROS_INFO("isAtPosition called...");

    geometry_msgs::PoseConstPtr sharedPose = this->getGroundTruthPosition();

    return fabs(sharedPose->position.x - x) < _ERROR_POSE &&
           fabs(sharedPose->position.y - y) < _ERROR_POSE &&
           fabs(sharedPose->position.z - z) < _ERROR_POSE - _Z_ERROR;
}