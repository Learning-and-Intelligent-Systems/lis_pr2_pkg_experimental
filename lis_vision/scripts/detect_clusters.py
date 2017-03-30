#!/usr/bin/env python
# Author: Ariel Anders
"""
this program filters clusters from /tabletop/clusters.  It finds
which clusters are large and meet a simple criteria to be on the table, 
then draws bounding boxes around the clusters
"""

import rospy
from object_recognition_msgs.msg import Table, TableArray
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
import pdb
import tf
from copy import deepcopy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from threading import Thread
from bounding_volumes import *

class ClusterFilter():
    table_frame = "head_mount_kinect_rgb_optical_frame"
    base_frame = "base_link"
    alpha = .5
    colors = [ ColorRGBA(r/255.,g/255.,b/255.,alpha) for (r,g,b) in \
            [ (154,4,110),  (46,189,89), (73,6,6), (180,71,71), \
            (207,208,110), (127,197,192), (23,34,75) ]]


    min_height = 0.3
    max_height = 1.5
    #min_w = 0.7
    #min_area = .5
    #max_area = .75
    density = 1e6

    def __init__(self, tf_listener=None):

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        self.best_table = None

        # keep track of head to base link continuously
        self.T = None
        self.transform_finder = Thread(target=self.updateTransform)
        self.transform_finder.start()

        self.pub_clusters = rospy.Publisher(\
                "/filtered_clusters", MarkerArray, queue_size=1)
        self.pub_bb = rospy.Publisher(\
                "/bounding_boxes", MarkerArray, queue_size=1)

        self.sub_table = rospy.Subscriber(\
                "/best_table", Table, self.cb_table, queue_size=1)
        self.sub = rospy.Subscriber(\
                "/tabletop/clusters", MarkerArray, self.cb_markers, queue_size=1, buff_size=2**24) # Need a large buff_size (https://github.com/ros/ros_comm/issues/536)
        rospy.loginfo("done init")        

    def cb_table(self, data_msg):
        self.best_table = data_msg
        #self.sub_table.unregister()

    def cb_markers(self, data_msg):
        if self.T is None or not self.best_table: return
        #T = self.T.copy()

        if len(data_msg.markers) == 0:
            return
        cluster = data_msg.markers[0]
        #header = self.best_table.header
        header = cluster.header

        try: 
            #self.tf_listener.waitForTransform(self.base_frame, header.frame_id, header.stamp, rospy.Duration(5.0))
            T = self.tf_listener.asMatrix(self.base_frame, header)

            #now = self.best_table.header.stamp - rospy.Duration(1.0)
            #now = rospy.Time.now() - rospy.Duration(1.0)
            #now = rospy.Time.now()
            #now = rospy.Time(0)
            #T = self.tf_listener.asMatrix(self.base_frame, Header(0,now,self.table_frame))

            #(trans,rot) = self.tf_listener.lookupTransform(self.table_frame, self.base_frame, rospy.Time.now())
            #(trans, rot) = self.tf_listener.lookupTransformFull(self.table_frame, now,
            #                          self.base_frame, past,
            #                          "/odom_combined")
        except Exception as e:
            print e

        # compute center of table position
        p = self.best_table.pose.position
        table = [float(q) for q in list(T*np.matrix([p.x,p.y,p.z,1.0]).T)[:3]]

        trans = trans_from_pose(ros_pose_to_or_pose(self.best_table.pose))
        hull_vertices = np.array([ros_point_to_np_point(point) for point in self.best_table.convex_hull])
        table_vertices = T.dot(trans).dot(np.vstack([hull_vertices.T, np.ones(len(hull_vertices))]))[:3,:].T
        tcenter, textents, tquat = aabb_from_points(table_vertices.T)
        (tx, ty, tz), (tw, th, tl) = tcenter, textents

        markers = [] 
        boxes = []
        results = []
        aabbs = []
        for i, cluster in enumerate(data_msg.markers):

            #self.tf_listener.transformPointCloud(self.base_frame, cluster)

            points = [[float(q) for q in \
                       list(T*np.matrix([p.x,p.y,p.z, 1.0]).T)[:3]] \
                       for p in cluster.points]

            ros_pts = [Point(*p) for p in points]
            points = np.array(points)

            # quick bounding box 
            center, extents, quat = aabb_from_points(points.T)
            (x, y, z), (w, h, l) = center, extents
            volume = w*h*l
            density = len(points)/volume

            # not a tight cluster of points
            if density < self.density: continue
            
            # box too large
            #if w*h*l > self.max_area: continue
            
            # box is above or below the table
            if abs((z-l/2.) - table[2]) > .05: continue

            # box is not over the table
            if not test_contains_2D((tcenter, textents), (center, extents)): continue

            # overlaps with previous boxes (should instead test how significant the overlap is)
            if any(test_overlap((center, extents), aabb) for aabb in aabbs): continue

            aabbs.append((center, extents))

            # robust bounding box
            center, extents, quat = robust_aabb_from_points(points.T, p=.98)
            (x, y, z), (w, h, l) = center, extents

            m = deepcopy(cluster) # head frame
            print m.header
            #m.header = Header(0, rospy.Time(0), self.base_frame)
            m.header = Header(0, rospy.Time.now(), self.base_frame)
            #m.header = Header(0, m.header.stamp, self.base_frame)
            print m.frame_locked
            m.frame_locked = False
            m.points = ros_pts
            m.ns = "clusters"
            bb.lifetime = rospy.Duration(1)

            bb = deepcopy(m)
            bb.type = Marker.CUBE
            bb.pose = Pose(Point(x,y,z), Quaternion(0,0,0,1))
            bb.scale.x, bb.scale.y, bb.scale.z = w, h, l
            bb.points = []
            bb.ns = "bounding_boxes"
            bb.lifetime = rospy.Duration(1)

            results.append((volume, x, y,  m, bb))

        results = sorted(results, \
                key=lambda x: (x[0], x[1], x[2]) , reverse=True)

        for i, (volume,x,y,m,bb) in enumerate(results):
            m.id = i
            m.color = self.colors[i%len(self.colors)]
            markers.append(m)

            bb.id = i
            bb.color = self.colors[i%len(self.colors)]
            boxes.append(bb)

            bbtext = deepcopy(bb)
            bbtext.id = i+100
            bbtext.type = Marker.TEXT_VIEW_FACING
            bbtext.text = "%d" % i
            bbtext.color = ColorRGBA(1,1,1,1)
            boxes.append(bbtext)

        self.pub_clusters.publish(MarkerArray(markers))
        self.pub_bb.publish(MarkerArray(boxes))

    def updateTransform(self):
        # continuously find transform for head to baselink
        while not rospy.is_shutdown():
            try:
                self.T = np.linalg.inv(self.tf_listener.asMatrix(self.table_frame,\
                        Header(0,rospy.Time(0),self.base_frame)))
            except:
                print "no tf"
                pass
            rospy.sleep(.1)

if __name__=="__main__":
    rospy.init_node("cluster_filters")
    tf=ClusterFilter()
    rospy.spin()
