#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <unistd.h>

#include "Uav.hpp"



// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline 
    Eigen::Vector3f convertFromString(StringView str)
    {
        // The next line should be removed...
        printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ',');
        if (parts.size() != 3)
        {
            throw RuntimeError("Invalid goto Input)");
        }
        else
        {
            Eigen::Vector3f outputVector;
            outputVector(0) = convertFromString<float>(parts[0]);
            outputVector(1) = convertFromString<float>(parts[1]);
            outputVector(2) = convertFromString<float>(parts[2]);
            return outputVector;
        }
    }
} // end namespace BT





BT::NodeStatus CheckBattery();



class MoveBase : public BT::AsyncActionNode, public Uav
{
  public:
    MoveBase(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {
      this->setName("pelican");
    }

    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return{ BT::InputPort<Eigen::Vector3f>("goto") }; }

    virtual void halt() override { _aborted = true; }

  private:
    bool _aborted;
};




class RotateBase : public BT::AsyncActionNode, public Uav
{
  public:
    RotateBase(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {
      this->setName("pelican");
    }

    virtual BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return{ BT::InputPort<float>("angle") }; }

    virtual void halt() override { _aborted = true; }

  private:
    bool _aborted;
};





// We want to wrap into an ActionNode the methods open() and close()
class UavInterface
{
  public:
    UavInterface(std::string uavName);

    BT::NodeStatus tookOff();

    BT::NodeStatus gotoPosition();

    BT::NodeStatus goAroundObject(int amountPoints, float distanceFromObject, Eigen::Vector3f& objectPosition, bool gotoRight=false);

    BT::NodeStatus rotateDegree(float angle);

  private:
    Uav uav;
};