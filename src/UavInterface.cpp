#include "UavInterface.hpp"





// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}




// You must override the virtual function tick()
BT::NodeStatus MoveBase::tick()
{
    std::cout << "MoveBase: " << this->name() << std::endl;
    _aborted = false;

    this->tookOff(2);

    Eigen::Vector3f goal;
    if (!getInput<Eigen::Vector3f>("goto", goal))
        throw BT::RuntimeError("Missing required input [goto]");


    this->gotoPositionRelative(goal(0), goal(1), goal(2));


    if (_aborted)
    {
        ROS_ERROR("Mission Aborted!");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("%s: Target Reached!", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
}




// You must override the virtual function tick()
BT::NodeStatus RotateBase::tick()
{
    std::cout << "RotateBase: " << this->name() << std::endl;
    _aborted = false;

    float angle;
    if (!getInput<float>("angle", angle))
        throw BT::RuntimeError("Missing required input [angle]");


    this->rotateDegree(angle);

    if (_aborted)
    {
        ROS_ERROR("Mission Aborted!");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("%s: Target Reached!", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
}









// Lazy declaration
void createTargetPoints(float angleRadians, float amountPoints, float distanceFromObject, Eigen::Vector3f& uavPosition, Eigen::Vector3f& objectPosition, Eigen::MatrixXf& outputPoints);

UavInterface::UavInterface(std::string uavName) 
{
    uav.setName(uavName);
}


BT::NodeStatus UavInterface::tookOff()
{
    uav.tookOff(2.0);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UavInterface::gotoPosition()
{
    uav.gotoPositionAbsolute(5, -5, 2);
    uav.gotoPositionRelative(-2, 2, 0);

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UavInterface::goAroundObject(int amountPoints, float distanceFromObject, Eigen::Vector3f& objectPosition, bool gotoRight)
{
    ROS_INFO("Calling goAroundObject...");
    float theta = float(TWO_PI) / amountPoints;

    if (gotoRight)
        theta = -theta;

    Eigen::Vector3f uavPosition = uav.getPosition();

    ROS_WARN("Current UAV Position (%.2f, %.2f, %.2f)", uavPosition(0), uavPosition(1), uavPosition(2));

    Eigen::MatrixXf outputPoints(amountPoints, 3);
    createTargetPoints(theta, amountPoints, distanceFromObject, uavPosition, objectPosition, outputPoints );

    // Moves the UAV to the points
    for (int row=0; row<outputPoints.rows(); row++)
    {
        ROS_INFO("Going to (%.2f, %.2f, %.2f)", outputPoints(row, 0), outputPoints(row, 1), outputPoints(row, 2));
        uav.gotoPositionAbsolute(outputPoints(row, 0), outputPoints(row, 1), outputPoints(row, 2));
    }

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UavInterface::rotateDegree(float angle)
{
    uav.rotateDegree(angle);
    return BT::NodeStatus::SUCCESS;
}


// Rotate and Translate a given 3D point
Eigen::Vector3f transformPoint(float angleRadians, Eigen::Vector3f& translation, Eigen::Vector3f& point)
{
    Eigen::Affine3f transformation(Eigen::AngleAxisf(angleRadians, Eigen::Vector3f::UnitZ()));

    return transformation * point;
}

// Create the path points based on the amountPoints, current uavPosition and objectPosition (Rotation and Translation)
void createTargetPoints(float angleRadians, float amountPoints, float distanceFromObject, Eigen::Vector3f& uavPosition, Eigen::Vector3f& objectPosition, Eigen::MatrixXf& outputPoints)
{
    for (int i=0; i<amountPoints; i++)
        outputPoints.block<1, 3>(i, 0) = transformPoint( angleRadians*(i+1), objectPosition, uavPosition );

    std::cout << "Target Points: \n" << outputPoints << "\n\n";
}