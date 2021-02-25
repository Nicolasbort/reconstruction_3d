#! /usr/bin/env python3

from helpers.yaml_parser import *
from Position import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import rospy
import time
import math
import tf


# Colors to print colored in terminal
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



class UAV():

    def __init__(self, uav_config_path):
        self.__loadConfigs(uav_config_path, print_configs=True)

        # Offset of the odometry position and target position
        self.__ERROR_POSE = 0.2

        # Max distance of a movement
        self.__MAX_STEP_SIZE = 0.5

        # Z axis error
        self.__Z_ERROR = 0.14



    def moveGlobalAndHeading(self, x, y, z, heading):
        
        odometry = rospy.wait_for_message(self.odometryTopic, Pose)

        steps = int( math.hypot( x-odometry.position.x, y-odometry.position.y ) / self.__MAX_STEP_SIZE )
        
        delta_x = (x - odometry.position.x) / steps
        delta_y = (y - odometry.position.y) / steps

        delta_heading = (heading - self.position.heading) / steps
        # rospy.logwarn("Heading steps %r degrees\n", delta_heading)

        for i in range(0, steps):
            target_x_delta = (self.position.x + delta_x*(i+1)) 
            target_y_delta = (self.position.y + delta_y*(i+1)) 
            target_heading_delta = self.position.heading + delta_heading + delta_heading*(i+1)
            # rospy.logwarn("Target heading steps %r degrees\n", target_heading_delta)
            self.gotoPosition(target_x_delta, target_y_delta, z, target_heading_delta)

        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.position.heading = heading


    def gotoPositionAndHeading(self, x, y, z, heading):
        
        odometry = self.getOdometryPosition()

        steps = int( math.hypot( x-odometry.position.x, y-odometry.position.y ) / self.__MAX_STEP_SIZE )
        
        # rospy.logwarn("STEPS: %d", steps)
        print(f"{bcolors.OKBLUE}Steps: {steps}{bcolors.ENDC}")

        if heading >= 6.28318:
            heading = 0

        delta_x = (x - odometry.position.x) / steps
        delta_y = (y - odometry.position.y) / steps
        delta_z = (z - odometry.position.z) / steps
        delta_heading = (heading - self.__getHeading(odometry.orientation)) / steps

        if abs(delta_x) < 0.001: delta_x=0
        if abs(delta_y) < 0.001: delta_y=0
        if abs(delta_z) < 0.001: delta_z=0

        print(f"\n{bcolors.OKBLUE}Axis Steps:{bcolors.ENDC}\n")
        print(f"{bcolors.OKBLUE}Dx = {delta_x}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Dy = {delta_y}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Dz = {delta_z}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Dheading = {delta_heading}{bcolors.ENDC}\n")

        # rospy.logwarn("Heading size per %r degrees\n", delta_heading)
        # print(f"{bcolors.OKBLUE}Heading step degree: %.2f{bcolors.ENDC}", delta_heading)


        for i in range(0, steps):
            # GLOBAL    (Not working)
            # dx = x - delta_x*(steps-i+1)
            # dy = y - delta_y*(steps-i+1)
            # self.gotoPosition(dx, dy, 2.0, delta_heading)

            # RELATIVE
            self.gotoPositionRelative(delta_x, delta_y, delta_z, delta_heading)


    def moveRelativeAndHeading(self, dx, dy, dz, dheading):

        target_x = self.position.x + dx
        target_y = self.position.y + dy
        target_z = self.position.z + dz
        target_heading = self.position.heading + dheading

        steps = int( math.hypot( target_x-self.position.x, target_y-self.position.y ) / self.__MAX_STEP_SIZE )
        rospy.logwarn("Steps %r\n", steps)


        delta_x = (target_x - self.position.x) / steps
        delta_y = (target_y - self.position.y) / steps
        delta_z = (target_z - self.position.z) / steps
        delta_heading = (target_heading - self.position.heading) / steps
        # rospy.logwarn("dx %r   dy %r   dz %r    heading %r degrees\n", delta_x, delta_y, delta_z, delta_heading)


        for i in range(0, steps):
            target_x_delta = (self.position.x + delta_x*(i+1)) 
            target_y_delta = (self.position.y + delta_y*(i+1)) 
            target_z_delta = (self.position.z + delta_z*(i+1))
            target_heading_delta = self.position.heading + delta_heading + delta_heading*(i+1)
            self.gotoPosition(target_x_delta, target_y_delta, target_z_delta, target_heading_delta)

        self.position.x = target_x
        self.position.y = target_y
        self.position.z = target_z
        self.position.heading = target_heading


    def getOdometryPosition(self):
        odometry = rospy.wait_for_message(self.odometryTopic, Pose, timeout=10)
        return odometry


    def setOdometryTopic(self, topic):
        self.odometryTopic = topic


    def setCameraTopic(self, topic):
        self.cameraTopic = topic


    def rotateDegree(self, angle):
        angle_radians = angle * (math.pi/180)

        rospy.logwarn("%d radians: %.3f", angle, angle_radians)
        self.rotateRadian(angle_radians)


    def rotateRadian(self, angle):
        odometry = self.getOdometryPosition()

        pose = PoseStamped()
        pose.pose.position.x = odometry.position.x
        pose.pose.position.y = odometry.position.y
        pose.pose.position.z = odometry.position.z

        yaw = self.__getHeading(odometry.orientation)

        relative_angle = yaw + angle

        rospy.logwarn("Current yaw: %.3f radians: %.3f. Final angle: %.3f", yaw, angle, relative_angle)


        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, relative_angle)


        rospy.logwarn("Quaternion: %r", quaternion)

        orientation = Quaternion(*quaternion)
        pose.pose.orientation = orientation

        self.posePublisher.publish(pose)

        time.sleep(8)


    def tookOff(self, height):
        self.gotoPositionRelative(0.0, 0.0, height, 0.0)
        time.sleep(2.5)



    #
    # Go to absolute position of the world
    #
    def gotoPosition(self, x, y, z, heading):   
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        print(f"\n{bcolors.WARNING}Publishing: {bcolors.ENDC}\n")
        print(f"{bcolors.WARNING}X = {x}{bcolors.ENDC}")
        print(f"{bcolors.WARNING}Y = {y}{bcolors.ENDC}")
        print(f"{bcolors.WARNING}Z = {z}{bcolors.ENDC}")

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, heading)
        orientation = Quaternion(*quaternion)
        pose.pose.orientation = orientation


        while not self.arrivedAtLocation(x, y, z):  
            # rospy.loginfo("Moving...") 
            self.posePublisher.publish(pose)
            time.sleep(0.8)        

        # rospy.loginfo("Arrived at the location...")  =

        odometry = self.getOdometryPosition()

        print(f"\n{bcolors.OKBLUE}Arrived. Axis Steps:{bcolors.ENDC}\n")
        print(f"{bcolors.OKBLUE}X = {odometry.position.x}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Y = {odometry.position.y}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Z = {odometry.position.z}{bcolors.ENDC}")
        print(f"{bcolors.OKBLUE}Yaw = {self.__getHeading(odometry.orientation)}{bcolors.ENDC}\n")

        time.sleep(0.8)        




    #
    # Moves relative to the uav position
    #
    def gotoPositionRelative(self, dx, dy, dz, dhd):


        odometry = self.getOdometryPosition()

        new_x = odometry.position.x + dx
        new_y = odometry.position.y + dy
        new_z = odometry.position.z + dz + self.__Z_ERROR
        new_hd = self.__getHeading(odometry.orientation) + dhd

        print(f"\n{bcolors.OKGREEN}Moving to:{bcolors.ENDC}\n")
        print(f"{bcolors.OKGREEN}X = {new_x}{bcolors.ENDC}")
        print(f"{bcolors.OKGREEN}Y = {new_y}{bcolors.ENDC}")
        print(f"{bcolors.OKGREEN}Z = {new_z}{bcolors.ENDC}")
        print(f"{bcolors.OKGREEN}Yaw = {new_hd}{bcolors.ENDC}\n")

        self.gotoPosition(new_x, new_y, new_z, new_hd)


    
    #
    # Returns True if the uav is close enough to the position
    #
    def arrivedAtLocation(self, x, y, z):
        position = self.getOdometryPosition().position
        return (abs(position.x-x) < self.__ERROR_POSE) and (abs(position.y-y) < self.__ERROR_POSE) and (abs(position.z-z) < self.__ERROR_POSE-0.05)




    def getPosition(self):
        position = self.getOdometryPosition().position
        rospy.logdebug("GET POSITION:  [%.2f, %.2f, %.2f]", position.x, position.y, position.z)
        return [position.x, position.y, position.z]

    
    #
    # Load the uav configuration from an yaml file
    #
    def __loadConfigs(self, config_path, print_configs=False):

        # Load yaml and return a dictionary
        configYaml = loadYaml(config_path)

        # UAV Position
        self.position = Position()
        self.position.x = configYaml["start_position"]["x"]
        self.position.y = configYaml["start_position"]["y"]
        self.position.z = configYaml["start_position"]["z"]
        self.position.heading = configYaml["start_position"]["heading"]

        # Topic Names
        self.odometryTopic = configYaml["topics"]["odometry"]
        self.cameraTopic = configYaml["topics"]["camera"]
        self.poseTopic = configYaml["topics"]["pose"]

        # UAV Publishers
        self.posePublisher = rospy.Publisher(self.poseTopic, PoseStamped, queue_size=10)

        # UAV Name
        self.uavName = configYaml["uav_name"]

        # Used to debug config informations
        if (print_configs):
            rospy.logwarn("UAV Position: (%.2f, %.2f, %.2f, %.3f)", self.position.x, self.position.y, self.position.z, self.position.heading)
            rospy.logwarn("UAV odometryTopic: %r", self.odometryTopic)
            rospy.logwarn("UAV cameraTopic: %r", self.cameraTopic)
            rospy.logwarn("UAV poseTopic: %r", self.poseTopic)
            rospy.logwarn("UAV posePublisher: %r", self.posePublisher)



    def __getHeading(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion (orientation_list)
        return yaw