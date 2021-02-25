#! /usr/bin/env python3

import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
from defines import *
from utils import *
from Uav import *

reconstruction_pkg_path = rospkg.RosPack().get_path('reconstruction_3d')

pelican_config_path = reconstruction_pkg_path + "/start/pelican_config.yaml"
hummingbird_config_path = reconstruction_pkg_path + "/start/hummingbird_config.yaml"

pelican = UAV(pelican_config_path)
humming = UAV(hummingbird_config_path)




def go_around_object(uav, amount_views, distance_from_object):
    
    delta_angle = (2*math.pi)/amount_views
    current_angle = delta_angle

    for i in range(0, amount_views):
        xl = distance_from_object * math.cos(current_angle) + STATUE_POSITION[0]
        yl = distance_from_object * math.sin(current_angle) + STATUE_POSITION[1]


        uav.moveGlobalAndHeading(xl, yl, 2.5, current_angle)

        rospy.logwarn("Going to the next position [%r, %r]", xl, yl)

        current_angle += delta_angle

        pub_save_img.publish(True)

    pub_save_img.publish(False)



def goto_half_object(uav, amount_views, distance_from_object, threshold_end=0, goto_left=True):
    '''
        Goes around the object with radius distance_from_object and stops at every view.
        threshold_end is the offset angle at the end of half circle used to the uavs don't
        crash at the end. goto_left is to choose if the uav must go to the left or right of
        the object
    '''

    rospy.logwarn("Starting Go to half object")

    delta_angle = (math.pi-threshold_end)/amount_views
    current_angle = delta_angle

    for i in range(0, amount_views):
        if (goto_left):
            xl = distance_from_object * math.cos(-current_angle) + STATUE_POSITION[0]
            yl = distance_from_object * math.sin(-current_angle) + STATUE_POSITION[1]
            uav.moveGlobalAndHeading(xl, yl, 2.5, -current_angle)
        else:
            xl = distance_from_object * math.cos(current_angle) + STATUE_POSITION[0]
            yl = distance_from_object * math.sin(current_angle) + STATUE_POSITION[1]
            uav.moveGlobalAndHeading(xl, yl, 2.5, current_angle)

        rospy.logwarn("Arrived at position [%r, %r]", xl, yl)

        current_angle += delta_angle

    rospy.logwarn("Go to half object finished")





def mission_planner_inspection_task(): 

    setup_vehicle()

    pelican.tookOff(2.0)

    goto_half_object(pelican, 5, 5, goto_left=True)

    rospy.loginfo("Finished!!")



if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    mission_planner_inspection_task()  

    rospy.spin()