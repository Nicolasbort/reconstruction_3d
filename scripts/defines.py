import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
from yaml_parser import *
import rospkg


# Get the path to the reconstruction package
# e.g: /home/user/catkin_ws/reconstruciton_3d
reconstruction_pkg_path = rospkg.RosPack().get_path('reconstruction_3d')

uav_pose_path = reconstruction_pkg_path     + "/start/uav_pose.yaml" 
object_pose_path = reconstruction_pkg_path + "/start/statue_pose.yaml" 
images_folder = reconstruction_pkg_path + "/images"

# Global variables to run the application
SIMULATION = True
ARR_THRES = 0.1
MAX_STEP_SIZE = 0.2
START_POSITION = getInitialPose(uav_pose_path) 
CURRENT_POSITION = START_POSITION
STATUE_POSITION = getObjectPose(object_pose_path)

set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
set_point_sim_pub = rospy.Publisher("/hydrone/command/pose", PoseStamped, queue_size=10)
pub_save_img = rospy.Publisher("/hydrone/camera/save_image", Bool, queue_size=10)