#!/usr/bin/env python
# Author: Caelan Garrett, Ariel Anders
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
from openravepy import quatFromRotationMatrix
from scipy.spatial import ConvexHull

def oobb_from_points_2D(points): # NOTE - not necessarily optimal
  mu = np.mean(points, axis=1)

  centered = (points - np.resize(mu, (3,1)))[:2]
  u, _, _ = np.linalg.svd(centered)
  if np.linalg.det(u) < 0: u[:,1] *= -1

  tformed = np.dot(u.T, centered)
  extents = np.max(tformed, axis=1) - np.min(tformed, axis=1)
  trans = np.identity(3)
  trans[:2,:2] = u
  w, h, l = np.max(points, axis=1) - np.min(points, axis=1)

  return mu, np.concatenate([extents, [l]]), quatFromRotationMatrix(trans)

def oobb_from_points(points): # NOTE - not necessarily optimal
  mean = np.mean(points, axis=1)

  mu = np.resize(np.mean(points, axis=1), (3,1))
  centered = points - mu
  u, _, _ = np.linalg.svd(centered)
  if np.linalg.det(u) < 0: u[:,1] *= -1

  # tf.Quaternion

  tformed = np.dot(u.T, centered)
  extents = np.max(tformed, axis=1) - np.min(tformed, axis=1)
  #trans = np.identity(3)
  trans = u

  return mean, extents, quatFromRotationMatrix(trans)

def aabb_from_points(points): # NOTE - not necessarily optimal
  mu = np.mean(points, axis=1)
  extents = np.max(points, axis=1) - np.min(points, axis=1)
  trans = np.array([0, 0, 0, 1])
  return mu, extents, trans

class ClusterFilter():
    table_frame = "head_mount_kinect_rgb_optical_frame"
    base_frame = "base_link"
    alpha = .5
    colors = [ ColorRGBA(r/255.,g/255.,b/255.,alpha) for (r,g,b) in \
            [ (154,4,110),  (46,189,89), (73,6,6), (180,71,71), \
            (207,208,110), (127,197,192), (23,34,75) ]]


    min_height= 0.3
    max_height=1.5
    min_w = 0.7
    min_area = .5
    max_area = .75
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
                "/tabletop/clusters", MarkerArray,self.cb_markers,queue_size=1)
        rospy.loginfo("done init")
        

    def cb_table(self,data_msg):
        self.best_table = data_msg
        #self.sub_table.unregister()

    def cb_markers(self, data_msg):
        if self.T is None: return
        #self.sub.unregister()

        good_clusters = []

        # compute center of table position
        if not self.best_table:
            table = (0,0,0)
        else:
            p = self.best_table.pose.position
            table =[ float(q) for q in \
                    list( self.T*np.matrix([p.x,p.y,p.z,1.0]).T)[:3]]

        markers = [] 
        boxes = []

        results = []
        for i, cluster in enumerate(data_msg.markers):
            points =[ [float(q) for q in \
                    list(self.T*np.matrix([p.x,p.y,p.z, 1.0]).T)[:3]] \
                    for p in cluster.points]

            ros_pts = [Point(*p) for p in points]
            points = np.array(points)

            print points.shape
            #(x, y, z), (w, h, l), quat = aabb_from_points(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points(points.T)
            (x, y, z), (w, h, l), quat = oobb_from_points_2D(points.T)
            print (x, y, z), (w, h, l)

            #x,y,z = np.mean(points, axis=0)
            #w,h,l = np.max(points,axis=0) - np.min(points,axis=0)
            #quat = np.array([0, 0, 0, 1])
            #print (x, y, z), (w, h, l)
            #print

            area = w*h*l
            density = len(points)/area

            # not a tight cluster of points
            if density < self.density: continue
            #box too large
            #if w*h*l > self.max_area: continue
            # box under table
            if z < table[2]: continue
            # box to high off table
            if z > (0.2 + table[2]): continue


            m= deepcopy(cluster)
            m.header = Header(0, rospy.Time(0), "base_link")
            m.points = ros_pts
            m.ns = "clusters"
            markers.append(m)

            bb = deepcopy(m)
            bb.type=Marker.CUBE
            bb.pose = Pose(Point(x,y,z), Quaternion(*quat))
            bb.scale.x = w
            bb.scale.y = h
            bb.scale.z = l
            bb.points = []
            bb.ns = "bounding_boxes"
            boxes.append(bb)

            results.append ( (area, x, y,  m, bb) )


        results = sorted(results, \
                key=lambda x: (x[0], x[1], x[2]) , reverse=True)


        for i, (area,x,y,m,bb) in enumerate(results):
            m.id = i
            bb.id = i
            bb.color = self.colors[i%len(self.colors)]
            m.color = self.colors[i%len(self.colors)]
            bbtext = deepcopy(bb)
            bbtext.id = i+100
            bbtext.type = Marker.TEXT_VIEW_FACING
            bbtext.text = "%d" % i
            bbtext.color = ColorRGBA(1,1,1,1)
            markers.append(m)
            boxes.append(bb)
            boxes.append(bbtext)




        self.pub_clusters.publish(MarkerArray(markers))
        self.pub_bb.publish(MarkerArray(boxes))

    def updateTransform(self):
        # continuously find transform for head to baselink
        while not rospy.is_shutdown():
            try:
                T =  self.tf_listener.asMatrix( self.table_frame,\
                        Header(0,rospy.Time(0),self.base_frame))
                T = np.linalg.inv(T)
                self.T = T
            except:
                print "no tf"
                pass
            rospy.sleep(1)

if __name__=="__main__":
    rospy.init_node("cluster_filters")
    tf=ClusterFilter()
    rospy.spin()




