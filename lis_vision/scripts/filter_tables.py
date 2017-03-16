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
import time
from std_msgs.msg import Header


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

def np_point(point):
    return np.array([point.x, point.y, point.z])

def np_quat(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    return np.arccos(np.clip(np.dot(unit_vector(v1), unit_vector(v2)), -1.0, 1.0))

class TableFilter():

    min_height= 0.5
    max_height= 0.9
    #min_w = 0.7
    min_area = 0.3
    #tform_sleep = .1

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
        #self.pub.publish(data_msg)
        #return

        t0 = time.time()
        good_tables = []
        
        """ # Do we need this?
        # find transform for head to baselink
        for i in range(5):
            try:
                #rospy.loginfo("looking for transform")
                ctime = self.tf_listener.getLatestCommonTime(\
                        deepcopy(data_msg.header.frame_id), 'base_link')
                break
            except:
                rospy.loginfo("trouble finding transform...")
                rospy.sleep(self.tform_sleep)
                ctime=None
        if ctime==None: return
        """

        max_table = None

        base_tfrom =  np.linalg.inv(self.tf_listener.asMatrix('head_mount_kinect_rgb_optical_frame',\
                Header(0,rospy.Time(0),'base_link')))
        for t in data_msg.tables:
            pose_stamp = PoseStamped(deepcopy(data_msg.header),deepcopy(t.pose))
            pose = self.tf_listener.transformPose('base_link', pose_stamp) # TODO - make this matrix multiply
            # TODO - do threaded

            tform = self.tf_listener.fromTranslationRotation(np_point(t.pose.position), 
                np_quat(t.pose.orientation)) # Not using pose.pose

            # check height is okay
            height = pose.pose.position.z
            if height < self.min_height or height > self.max_height:
                continue

            # check orientation is okay
            #if pose.pose.orientation.w < self.min_w:
            #    continue
            normal = np_quat(pose.pose.orientation)[:3]
            if normal[2] < 0: normal *= -1
            if angle_between(normal, np.array([0, 0, 1])) > np.pi/8:
                continue


            #points = np.array([[p.x,p.y,p.z] for p in t.convex_hull])
            points = base_tfrom.dot(tform).dot(np.array([[p.x,p.y,p.z,1] for p in t.convex_hull]).T).T[:,:3]
            #x,y,z = (np.max(points,axis=0) + np.min(points,axis=0))/2
            w,h,l = np.max(points,axis=0) - np.min(points,axis=0)
            if w*h < self.min_area or l > .1:
                continue

            #print x,y,z, w,h,l
            #print pose.pose
            #print tf.transformations.euler_from_quaternion(np_quat(pose.pose.orientation))

            if not max_table or (max_table[1] < w*h):
                max_table = t, w*h

            # passed tests
            good_tables.append(t)

        result = TableArray()
        result.header = data_msg.header
        result.header.stamp = rospy.Time.now()
        result.tables = good_tables
        self.pub.publish(result)

        if max_table:            
            result = Table()
            result.header = data_msg.header
            result.header.stamp = rospy.Time.now()
            result.pose = max_table[0].pose
            result.convex_hull = max_table[0].convex_hull
            self.pub_single.publish(result)
        #print 'Filter table:', time.time() - t0, len(good_tables), max_table is not None


if __name__=="__main__":
    rospy.init_node("filter_tables")
    TableFilter()
    rospy.spin()




