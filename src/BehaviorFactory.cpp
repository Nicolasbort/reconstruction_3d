#include <behaviortree_cpp_v3/bt_factory.h>
#include "UavInterface.hpp"


// Timing macros
#define MICROSECONDS(x) std::chrono::microseconds(x)
#define MILLISECONDS(x) std::chrono::milliseconds(x)
#define SECONDS(x) std::chrono::seconds(x)


// Help function to wrong command line inputs
void help();


using namespace BT;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        help();
        return -1;
    }

    ros::init(argc, argv, "behavior_tree_node");

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    Eigen::Vector3f objectPosition;
    objectPosition << 0.0, 0.0, 0.0;


    PortsList uavInterfacePorts = { InputPort<std::string>("angle") };

    // You can also create SimpleActionNodes using methods of a class
    // Note: the name used to register should be the same used in the XML.
    UavInterface uavInterface("pelican");
    factory.registerSimpleAction("TookOff",         std::bind(&UavInterface::tookOff,        &uavInterface));
    factory.registerSimpleAction("GoTo",            std::bind(&UavInterface::gotoPosition,   &uavInterface));
    factory.registerSimpleAction("GoAroundObject",  std::bind(&UavInterface::goAroundObject, &uavInterface, 10, 5, objectPosition, false));
    factory.registerSimpleAction("RotateDegree",    std::bind(&UavInterface::rotateDegree,   &uavInterface, 90));

    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    Tree tree = factory.createTreeFromFile(argv[1]);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}


void help()
{
    std::cout <<
    "ROS Usage: rosrun rec3d behavior_tree_exe <PATH_TO_TREE.xml>" << std::endl <<
    "Usage:     ./behavior_tree_exe <PATH_TO_TREE.xml>"   << std::endl;
}