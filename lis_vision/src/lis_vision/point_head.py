#!/usr/bin/env python
import rospy
# Author Ariel Anders
"""
this function listens to rviz Publish point message to send 
a point head goal to the robot 
"""

from geometry_msgs.msg import Point, PointStamped
from actionlib import SimpleActionClient as SAC
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction

class PointHead:
    def __init__(self):
        
        self.client = SAC("/head_traj_controller/point_head_action",\
                PointHeadAction)
        rospy.loginfo("initliazing head action client")
        self.client.wait_for_server()
        self.sub = rospy.Subscriber("/clicked_point",\
                 PointStamped, self.cb, queue_size=1)
        rospy.loginfo("head tracker initialized")

    def cb(self, msg):
        rospy.loginfo("recieved a point head command! moving")
        goal = PointHeadGoal()

        goal.target = msg
        goal.pointing_frame="/high_def_frame"
        goal.pointing_axis = Point(1,0,0)
        goal.min_duration=rospy.Duration(.15)
        goal.max_velocity = 1.0 # rad/s

        self.client.send_goal(goal)
        #result = self.client.wait_for_result(rospy.Duration(1))
        #rospy.loginfo("action done. Result is %d " % result)

if __name__=="__main__":
    rospy.init_node("head_pointer")
    ph = PointHead()
    rospy.spin()



