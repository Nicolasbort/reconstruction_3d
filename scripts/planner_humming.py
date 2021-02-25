#! /usr/bin/env python3

from uav_utils import *


hummingbird_config_path = reconstruction_pkg_path + "/start/hummingbird_config.yaml"

humming = UAV(hummingbird_config_path)

pub_start_reconstruction = rospy.Publisher("/reconstruction/start", Bool, queue_size=10)


def mission_planner_inspection_task(): 

    setup_vehicle()

    humming.tookOff(2.0)

    goto_half_object(humming, 5, 5, STATUE_POSITION, threshold_end=0.14159, goto_left=True)

    pub_start_reconstruction.publish(True)

    rospy.loginfo("Finished Pelican")


if __name__ == "__main__": 
    rospy.init_node("humming_planner", anonymous=False)    

    mission_planner_inspection_task()  

    rospy.spin()