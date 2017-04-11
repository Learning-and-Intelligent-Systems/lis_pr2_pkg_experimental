import rospy
from lis_pr2_pkg.uber_controller import Uber
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

rospy.init_node("move_test")

uc = Uber()

arm = 'r'
frame='base_link'

pub  = rospy.Publisher("r_cart/command_pose", PoseStamped, queue_size=1)

# get current pose
def get_current_pose():
    return uc.return_cartesian_pose(arm, frame)

# stamp a pose to be a command
def stamp_pose( (pos,quat)):
    ps = PoseStamped(
            Header(0, rospy.Time(0), frame),\
            Pose ( Point(*pos), Quaternion(*quat))
            )
    return ps


# move relative 
def move_relative(x,y,z):
    pos,quat = get_current_pose()
    pos[0] += x
    pos[1] += y
    pos[2] += z

    cmd = stamp_pose( (pos,quat) )
    pub.publish(cmd)

    print "moving to ", cmd

for i in range(10):
    move_relative( .05,0,.07)
    raw_input()

for i in range(10):
    move_relative( -.05,0,-.07)
    raw_input()

