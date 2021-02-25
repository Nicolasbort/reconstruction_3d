#! /usr/bin/env python3

import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
import rospkg
import math
from helpers.yaml_parser import *
import numpy as np
from Uav import UAV 


# Whether or not will plot the uav 2D trajectory around the object
PLOT_TRAJECTORY = True
SIMULATION = True


if PLOT_TRAJECTORY:
    import matplotlib.pyplot as plt


pub_save_img = rospy.Publisher("/hydrone/camera/save_image", Bool, queue_size=10)

try:
    reconstruction_pkg_path = rospkg.RosPack().get_path('reconstruction_3d')
except rospkg.ResourceNotFound:
    print("Rospkg 'reconstruction_3d' not found. Can't read the object position\n")
    sys.exit()
    # reconstruction_pkg_path = "/home/nicolas/catkin_ws/src/reconstruction_3d"
object_pose_path = reconstruction_pkg_path + "/start/statue_pose.yaml" 


STATUE_POSITION = getObjectPose(object_pose_path)


def setup_vehicle():
    global SIMULATION

    if not SIMULATION: call_set_mode("GUIDED", 4)
    print("Mode guided set")

    time.sleep(1)

    if not SIMULATION: rospy.set_param("/mavros/vision_pose/tf/listen", True)

    time.sleep(5)

    if not SIMULATION: call_arming(True)
    print("First Arming Check")

    time.sleep(5)



def go_around_object(uav, amount_views, distance_from_object, object_position, goto_right=False):
    
    theta = (2*math.pi)/amount_views

    if goto_right:
        theta = -theta

    current_angle = theta

    list_points = create_target_points(theta, amount_views, uav.getPosition(), distance_from_object, object_position)


    for path_point in list_points:

        uav.gotoPositionAndHeading(path_point[0], path_point[1], path_point[2], -current_angle)

        rospy.logwarn("Going to the next position [%.2f, %.2f, %.2f, %.3f]", path_point[0], path_point[1], path_point[2], -current_angle)

        current_angle += theta

        # pub_save_img.publish(True)

    # # pub_save_img.publish(False)



def goto_half_object(uav, amount_views, distance_from_object, object_position, threshold_end=0, goto_left=True):
    '''
        Goes around the object with radius distance_from_object and stops at every view.
        threshold_end is the offset angle at the end of half circle used to the uavs don't
        crash at the end. goto_left is to choose if the uav must go to the left or right of
        the object

        Parameters
        ----------
        uav : UAV
            Instance of UAV class
        amount_views: Int
            Number of checkpoints in the circle around the object
        distance_from object: Int
            Radius of the rotation around the object
    '''

    rospy.logwarn("Starting Go to half object")

    theta = (math.pi-threshold_end)/amount_views

    if (goto_left):
        theta = -theta

    list_points = create_target_points(theta, amount_views, uav.getPosition(), distance_from_object, object_position)
        
    current_angle = theta
                        
    # rotation2D = np.array([[np.cos(theta), -np.sin(theta)],
    #                     [np.sin(theta), np.cos(theta)]])

    # translation2D = np.array([object_position[0], object_position[1]])

    for point in list_points:

        # uav.gotoPositionAndHeading(point[0], point[1], point[2], -current_angle)
        uav.moveGlobalAndHeading(point[0], point[1], point[2], -current_angle)

        rospy.logwarn("Going to the next position [%.2f, %.2f, %.2f]", point[0], point[1], point[2])

        current_angle += theta



    # for i in range(0, amount_views):

    #     # Current uav position
    #     xy = np.array([uav.position.x, uav.position.y])

    #     # New uav position
    #     xyl = np.dot(xy, rotation2D) + translation2D

    #     uav.moveGlobalAndHeading(xyl[0], xyl[1], 2.5, -current_angle)

    #     rospy.logwarn("%r Arrived at position [%.2f, %.2f]", uav.uavName, xyl[0], xyl[1])
        
    #     pub_save_img.publish(True)
    #     time.sleep(1)
    #     pub_save_img.publish(False)

    #     current_angle += theta

    rospy.logwarn("Go to half object finished")




def goto_start_position(uav):
    pass


def create_target_points(radians_angle, amount_points, start_position, distance_from_object, object_position):

    list_points = []

    # Matplotlib
    if PLOT_TRAJECTORY:
        # Point of the center of the object
        plt.plot(object_position[0], object_position[1], 'ro')
        # Initializing axis
        x_axis = []
        y_axis = []

    start_position = np.array(start_position)

    for i in range(amount_points):

        rotated_position = rotate(object_position, start_position, radians_angle*(i+1))

        list_points.append(rotated_position)

        # Matplotlib
        if PLOT_TRAJECTORY:
            x_axis.append(rotated_position[0])
            y_axis.append(rotated_position[1])

    
    # Matplotlib
    if PLOT_TRAJECTORY:
        # print("\nX_AXIS:\n", x_axis)  
        # print("\nY_AXIS:\n", y_axis)   

        plt.plot(x_axis, y_axis, '*')
        plt.ylabel('Y')
        plt.xlabel('X')
        plt.show()


    return list_points


def rotate(object_position, point_xyz, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """

    rotation3D = np.array([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle),  0],
                            [0,             0,              1]])

    translation3D = np.array([object_position[0], object_position[1], 0.0])

    point_rotated = np.dot(point_xyz, rotation3D) + translation3D

    return point_rotated