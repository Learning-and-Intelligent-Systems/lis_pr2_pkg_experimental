#!/usr/bin/env python
import rospy
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject, DisplayTrajectory
import pdb
import numpy as np



class SimpleManipulation:
    def __init__(self, arm="r"):

        self.passThrough = True
        self.pub_co = rospy.Publisher("/collision_object", \
                CollisionObject, queue_size=1)

        rospy.loginfo("setting up ... please wait")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = arm
        arm_name = "right" if "r" in arm else "left"
        self.group = moveit_commander.MoveGroupCommander("%s_arm"%arm_name)
        self.dt_pub = rospy.Publisher(\
                '/move_group/display_planned_path', DisplayTrajectory)

    def plan(self):
        dt = DisplayTrajectory()
        dt.trajectory_start = self.robot.get_current_state()
        plan = self.group.plan()
        dt.trajectory= [plan]
        self.dt_pub.publish(dt)
        return plan

    def go(self, plan=None):
        if plan == None:
            plan = self.plan()
        q = raw_input("go?")
        self.group.go(wait=True)


    def home(self):
        
        home= {
        'l':[ 0.95, 0.0, np.pi/2., -np.pi/2., -np.pi*1.5, -np.pi/2.0, np.pi],
        'r':[ -0.7, 0.0, -np.pi/2., -np.pi/2., np.pi*1.5, -np.pi/2.0, np.pi]
            }
        try:
            self.group.set_joint_value_target(home[self.arm])
        except:
            pass # ignore joint angle warning since it always happens
        self.go()


        
    def start_grasp(self):
        self.home()

        while not rospy.is_shutdown():
            self.sub_co = rospy.Subscriber("/ari_collision_object", CollisionObject, self.cb_passthrough, queue_size=1)
            self.passThrough = True
            raw_input("hit enter when 'bb' is ready")
            self.sub_co.unregister()
            self.passThrough = False

            #open gripper
            print "trying to pick"
            self.group.pick("bb")
            print "returning to start"

            self.home()



    def cb_passthrough(self, msg):
        if self.passThrough:
            self.pub_co.publish(msg)

if __name__=="__main__":
    rospy.init_node("arimovetest")
    sm = SimpleManipulation()
    sm.start_grasp()






"""

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("right_arm")
display_trajectory_publisher = rospy.Publisher(
                              '/move_group/display_planned_path',
                              moveit_msgs.msg.DisplayTrajectory)
pose_target = group.get_current_pose()
pose_target.pose.position.z += 0.05
group.set_pose_target(pose_target)

plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."

display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);


# Uncomment below line when working with a real robot
q = raw_input("go?")
if q in ["y", "yes", "Y", "YES", "Yes"]:
    group.go(wait=True)


r

"""
