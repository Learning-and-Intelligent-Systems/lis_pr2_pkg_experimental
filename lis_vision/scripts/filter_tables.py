#!/usr/bin/env python
# Author: Ariel Anders
"""
This program removes detected tables that are too small or too low/high to be 
a table I'm interested in.
"""

import rospy
from object_recognition_msgs.msg import Table, TableArray
from geometry_msgs.msg import PoseStamped
import pdb
import tf
from copy import deepcopy
import numpy as np



"""
TableArray.msg


    Header header

    # Just an array of tables
    object_recognition_msgs/Table[] tables

Table.msg

    # Informs that a planar table has been detected at a given location

    Header header

    # The pose gives you the transform that take you to the coordinate system
    # of the table, with the origin somewhere in the table plane and the 
    # z axis normal to the plane
    geometry_msgs/Pose pose

    # There is no guarantee that the table does NOT extend further than the
    # convex hull; this is just as far as we've observed it.
    # The origin of the table coordinate system is inside the convex hull

    # Set of points forming the convex hull of the table
    geometry_msgs/Point[] convex_hull

"""

class TableFilter():

    min_height= 0.3
    max_height=1.5
    min_w = 0.7
    min_area = .5

    def __init__(self, tf_listener=None):

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.pub = rospy.Publisher(\
                "/filtered_table_array", TableArray, queue_size=1)
        self.pub_single = rospy.Publisher(\
                "/best_table", Table, queue_size=1)
        self.sub = rospy.Subscriber(\
                "/table_array", TableArray, self.cb_table, queue_size=1)
        rospy.loginfo("done init")

    def cb_table(self,data_msg):
        #self.sub.unregister()

        good_tables = []
        

        # find transform for head to baselink
        for i in range(5):
            try:
                #rospy.loginfo("looking for transform")
                time = self.tf_listener.getLatestCommonTime(\
                        deepcopy(data_msg.header.frame_id), 'base_link')
                break
            except:
                rospy.loginfo("trouble finding transform...")
                rospy.sleep(1)
                time=None
        if time==None: return
        
        max_table = None
            
        for t in data_msg.tables:
            pose_stamp = PoseStamped(deepcopy(data_msg.header),deepcopy(t.pose))

            pose = self.tf_listener.transformPose('base_link', pose_stamp)

            # check height is okay
            height = pose.pose.position.z
            # print height, pose.pose.orientation.w

            if height < self.min_height or height > self.max_height:
                continue

            # check orientation is okay
            if pose.pose.orientation.w < self.min_w:
                continue

            points = np.array([[p.x,p.y,p.z] for p in t.convex_hull])
            w,h,l = np.max(points,axis=0) - np.min(points,axis=0)


            if w*h < self.min_area:
                continue

            if not max_table or (max_table[1] < w*h):
                max_table = t, w*h

            # passed tests
            good_tables.append(t)

    
        if max_table:
            result = TableArray()
            result.header = data_msg.header
            result.header.stamp = rospy.Time.now()
            result.tables = good_tables
            self.pub.publish(result)

            
            result = Table()
            result.header = data_msg.header
            result.header.stamp = rospy.Time.now()
            result.pose = max_table[0].pose
            result.convex_hull = max_table[0].convex_hull
            self.pub_single.publish(result)



if __name__=="__main__":
    rospy.init_node("filter_tables")
    tf=TableFilter()
    rospy.spin()




