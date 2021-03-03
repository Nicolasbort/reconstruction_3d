#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include "UavInterface.hpp"



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

    ros::init(argc, argv, "main_tree_node");

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    // factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<RotateBase>("RotateBase");

    Tree tree = factory.createTreeFromFile(argv[1]);




    while( tree.tickRoot() == NodeStatus::RUNNING )
    {
        std::this_thread::sleep_for( MILLISECONDS(10) );
    }

    // while(ros::ok() && tree.tickRoot() == NodeStatus::RUNNING)
    // {
    //     std::this_thread::sleep_for(SECONDS(3));
    //     ROS_WARN("Sending another tick...");
    // }


    return 0;
}


void help()
{
    std::cout <<
    "ROS Usage: rosrun rec3d main_tree_exe <PATH_TO_TREE.xml>" << std::endl <<
    "Usage:     ./main_tree_exe <PATH_TO_TREE.xml>"   << std::endl;
}