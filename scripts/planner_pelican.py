#! /usr/bin/env python3

from uav_utils import *


pelican_config_path = reconstruction_pkg_path + "/start/pelican_config.yaml"


def mission_planner_inspection_task(): 

    setup_vehicle()

    pelican = UAV(pelican_config_path)

    pelican.tookOff(2.0)

    # pelican.gotoPositionRelative(-2.13, -0.4, 0.0, 0.0)
    # pelican.gotoPositionRelative(-3.0, 0.0, 0.0, 0.0)
    # pelican.gotoPositionRelative(0.0, -1.0, 0.0, 0.0)
    # pelican.gotoPositionRelative(5.0, 0.0, 0.0, 0.0)

    # pelican.gotoPosition(3.0, 0.0, 2.0, 0.0)
    # pelican.gotoPosition(4.0, 0.0, 2.0, 0.0)
    # pelican.gotoPosition(4.0, 0.0, 1.0, 0.0)

    # pelican.rotateRadian(0.628)

    # pelican.rotateDegree(-270)

    # pelican.rotateDegree(-90)

    # pelican.gotoPositionAndHeading(0, -5, 2, -1.57)

    # pelican.moveGlobalAndHeading(-1.0, -4.0, 2.0, -1.5)
    # pelican.moveGlobalAndHeading(-1.5, -3.0, 2.0, -1.5)

    # go_around_object(pelican, 5, 5, STATUE_POSITION)

    VIEW_POINTS = 10
    DISTANCE_FROM_OBJECT = 6

    go_around_object(pelican, VIEW_POINTS, DISTANCE_FROM_OBJECT, STATUE_POSITION, goto_right=False)

    rospy.loginfo("Finished Pelican")


if __name__ == "__main__": 
    rospy.init_node("pelican_planner", anonymous=False)    

    mission_planner_inspection_task()  

    rospy.spin()

