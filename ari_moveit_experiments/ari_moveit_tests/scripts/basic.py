#!/usr/bin/env python
import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pdb
# this program will move the right arm up 0.05m



rospy.init_node("arimovetest")
rospy.loginfo("loading stuff... takes awhile")
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
q = raw_input("type 'y' to go: ")
if q in ["y", "yes", "Y", "YES", "Yes"]:
    print 'Executing trajecotry'
    group.go(wait=True)
else:
    print 'Terminating'
print("exiting error")
