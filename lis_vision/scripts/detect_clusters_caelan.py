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
from sensor_msgs.msg import CameraInfo
import pdb
import tf
from copy import deepcopy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from threading import Thread
from openravepy import quatFromRotationMatrix
#from scipy.spatial import ConvexHull
from time import time
from manipulation.bodies.geometry import convex_hull

from openravepy import Environment
from manipulation.bodies.bodies import box_body
from manipulation.primitives.transforms import set_pose, set_point, get_point, trans_from_pose, trans_transform_points
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from manipulation.constants import PARALLEL_LEFT_ARM, FOLDED_LEFT_ARM, TOP_HOLDING_LEFT_ARM, \
    SIDE_HOLDING_LEFT_ARM, REST_LEFT_ARM, WIDE_HOLDING_LEFT_ARM, FAR_HOLDING_LEFT_ARM
from manipulation.primitives.utils import mirror_arm_config

from lis_pr2_pkg.uber_controller import Uber
from pick_and_place import feasible_pick_place, get_grasps, initialize_openrave, centered_box_body
from math import atan2
from filter_tables import angle_between

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from image_geometry import PinholeCameraModel

from bounding_volumes import *

# def oobb_from_points_2D(points): # NOTE - not necessarily optimal
#   mu = np.mean(points, axis=1)
#
#   centered = (points - np.resize(mu, (3,1)))[:2]
#   u, _, _ = np.linalg.svd(centered)
#   if np.linalg.det(u) < 0: u[:,1] *= -1
#
#   tformed = np.dot(u.T, centered)
#   extents = np.max(tformed, axis=1) - np.min(tformed, axis=1)
#   trans = np.identity(3)
#   trans[:2,:2] = u
#   w, h, l = np.max(points, axis=1) - np.min(points, axis=1)
#
#   return mu, np.concatenate([extents, [l]]), or_quat_to_ros_quat(quatFromRotationMatrix(trans))

def robust_oobb_from_points_2D_convex(points): # NOTE - not necessarily optimal
  points_2D = points[:,:2]
  vertices, edges = convex_hull(points_2D)
  min_center, min_rot, min_extents = None, None, None
  for i1, i2 in edges:
    x, y = vertices[i2] - vertices[i1]
    angle = atan2(y, x)
    rot = rot_2D(angle)
    rot_vertices = np.dot(rot.T, points_2D.T)

    max_points = robust_max(rot_vertices)
    min_points = robust_min(rot_vertices)

    center = np.dot(rot, (max_points + min_points)/2).T
    diff = max_points - min_points
    extents = np.array([diff[0,0], diff[1,0]]) # Do this differently
    if min_extents is None or np.prod(extents) < np.prod(min_extents):
        min_center, min_rot, min_extents = center, rot, extents

  center = (np.max(points, axis=0) + np.min(points, axis=0))/2
  center[:2] = min_center
  extents = np.max(points, axis=0) - np.min(points, axis=0)
  extents[:2] = min_extents
  trans = np.identity(3)
  trans[:2,:2] = min_rot

  return center, extents, or_quat_to_ros_quat(quatFromRotationMatrix(trans))

def oobb_from_points_2D_convex(points): # NOTE - not necessarily optimal
  vertices, edges = convex_hull(points[:, :2])
  min_center, min_rot, min_extents = None, None, None
  for i1, i2 in edges:
    x, y = vertices[i2] - vertices[i1]
    angle = atan2(y, x)
    rot = rot_2D(angle)
    rot_vertices = np.dot(rot.T, vertices.T)
    center = np.dot(rot, (np.max(rot_vertices, axis=1) + np.min(rot_vertices, axis=1))/2).T
    diff = np.max(rot_vertices, axis=1) - np.min(rot_vertices, axis=1)
    extents = np.array([diff[0,0], diff[1,0]]) # Do this differently
    if min_extents is None or np.prod(extents) < np.prod(min_extents):
        min_center, min_rot, min_extents = center, rot, extents

  center = (np.max(points, axis=0) + np.min(points, axis=0))/2
  center[:2] = min_center
  extents = np.max(points, axis=0) - np.min(points, axis=0)
  extents[:2] = min_extents
  trans = np.identity(3)
  trans[:2,:2] = min_rot

  return center, extents, or_quat_to_ros_quat(quatFromRotationMatrix(trans))

def oobb_from_points_2D(points): # NOTE - not necessarily optimal
  mu = np.mean(points, axis=1)
  centered = points - np.resize(mu, (3,1))
  u, _, _ = np.linalg.svd(centered[:2])
  if np.linalg.det(u) < 0: u[:,1] *= -1

  trans = np.identity(3)
  trans[:2,:2] = u
  tformed = np.dot(trans.T, centered)
  extents = np.max(tformed, axis=1) - np.min(tformed, axis=1)

  return mu, extents, or_quat_to_ros_quat(quatFromRotationMatrix(trans))

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

  return mean, extents, or_quat_to_ros_quat(quatFromRotationMatrix(trans))

def oobb_from_points2(points): # NOTE - has no motivation to orient z correctly
    import trimesh # NOTE - trimesh only works in 3D
    mesh = trimesh.convex.convex_hull(points)
    print 'Vertices:', len(mesh.vertices)
    oobb = mesh.bounding_box_oriented
    trans = oobb.primitive.transform
    return trans[:3, 3], oobb.primitive.extents, or_quat_to_ros_quat(quatFromRotationMatrix(trans[:3,:3]))

# env = Environment()
# #robot = env.ReadRobotXMLFile('robots/pr2-beta-sim.robot.xml') # ReadRobotData
# robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
# env.Add(robot)
# initialize_openrave(env)
# quit()

LEFT_TOP_SIDE = [1.5578610251557174, -0.13460793687920436, 1.6131468003997858, -1.473015195165653, 95.71201614935508, -1.4192964178436847, 37.862692851419574]
RIGHT_TOP_SIDE = [-1.5822931843358639, -0.08986685225797848, -1.4494837934373954, -1.4676586788444625, 29.969636385648396, -1.6421113802057423, -75.69328304787416]

class ClusterFilter():
    head_frame = "head_mount_kinect_rgb_optical_frame"
    base_frame = "base_link"
    world_frame = "odom_combined"
    display_frame = base_frame

    alpha = .5
    colors = [ ColorRGBA(r/255.,g/255.,b/255.,alpha) for (r,g,b) in \
            [ (154,4,110),  (46,189,89), (73,6,6), (180,71,71), \
            (207,208,110), (127,197,192), (23,34,75) ]]
    duration = 2
    tfrom_sleep = .1


    min_height= 0.3
    max_height=1.5
    min_w = 0.7
    min_area = .5
    max_area = .75
    density = 1e6

    min_delta = .05
    arm = 'l'
    max_clusters = 5

    def __init__(self, tf_listener=None):
        self.other_arm = 'r' if self.arm == 'l' else 'l'

        self.uc = Uber()
        self.detected = False

        self.env = Environment()
        #self.robot = self.env.ReadRobotXMLFile('robots/pr2-beta-sim.robot.xml') # ReadRobotData
        self.robot = self.env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
        self.env.Add(self.robot)
        initialize_openrave(self.env, arm=self.arm, min_delta=self.min_delta)

        # TODO - mirror config is definitely not working

        print self.uc.get_torso_pose()
        print self.arm, self.uc.get_joint_positions(self.arm)
        print self.other_arm, self.uc.get_joint_positions(self.other_arm)

        dt = 4.0
        #self.uc.look_down_center(blocking=False)
        self.uc.command_head([0, np.pi/3.], dt, blocking=False)

        self.uc.command_torso(0.3, blocking=False, timeout=2)
        self.uc.open_gripper(self.arm, blocking=False)
        #self.uc.close_gripper(self.other_arm, blocking=False)
        self.uc.open_gripper(self.other_arm, blocking=False)
        self.uc.command_joint_pose('l', LEFT_TOP_SIDE, dt, blocking=False) # 
        self.uc.command_joint_pose('r', RIGHT_TOP_SIDE, dt, blocking=True) # REST_LEFT_ARM
        #self.uc.command_joint_pose('r', mirror_arm_config(FOLDED_LEFT_ARM ), dt, blocking=True) # REST_LEFT_ARM

        self.robot.SetDOFValues(np.array([0.54800022]), self.robot.GetActiveManipulator().GetGripperIndices())
        self.robot.SetDOFValues(np.array([self.uc.get_torso_pose()]), [self.robot.GetJointIndex('torso_lift_joint')])
        self.robot.SetDOFValues(self.uc.get_joint_positions('l'), self.robot.GetManipulator('leftarm').GetArmIndices())
        self.robot.SetDOFValues(self.uc.get_joint_positions('r'), self.robot.GetManipulator('rightarm').GetArmIndices())

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        self.best_table = None
        self.camera_info = None

        # keep track of head to base link continuously
        self.T = None
        self.transform_finder = Thread(target=self.updateTransform)
        self.transform_finder.start()
        self.cam_model = PinholeCameraModel() # TODO - use pointcloud or stero camera?

        self.pub_clusters = rospy.Publisher(\
                "/filtered_clusters", MarkerArray, queue_size=1)
        self.pub_bb = rospy.Publisher(\
                "/bounding_boxes", MarkerArray, queue_size=1)

        self.sub_kinect = rospy.Subscriber(\
                "/head_mount_kinect/rgb/camera_info", CameraInfo, self.cb_kinect, queue_size=1) # 'depth' | 'ir'
        self.sub_table = rospy.Subscriber(\
                "/best_table", Table, self.cb_table, queue_size=1)
        self.sub = rospy.Subscriber(\
                "/tabletop/clusters", MarkerArray,self.cb_markers,queue_size=1)
        rospy.loginfo("done init")

        """
        self.sound_handle = SoundClient()
        rospy.sleep(1)
        #self.sound_handle.stopAll()
        #voice = 'voice_kal_diphone'
        #s = "I like mudkip" # Fake news
        #self.sound_handle.say(s, voice)
        #rospy.loginfo(s)

        filename = "/home/caelan/Downloads/magic.wav"
        #filename = "say-beep.wav"
        self.sound_handle.playWave(filename)
        # https://www.onlinevideoconverter.com/
        rospy.sleep(1)
        """

        # /opt/ros/indigo/share/sound_play/sounds/

    def cb_kinect(self, data_msg):
        #if self.camera_info is None:
        self.camera_info = data_msg
        self.cam_model.fromCameraInfo(self.camera_info)


        self.max_range = rospy.get_param("/head_mount_kinect/disparity_depth/max_range")
        self.min_range = rospy.get_param("/head_mount_kinect/disparity_depth/min_range")
        
        h, w = self.camera_info.height, self.camera_info.width
        pixels = [
            (0, 0),
            (h, 0),
            (0, w),
            (h, w),
            #(h/2, w/2),

            (h/2, 0),
            (h/2, w),
            (0, w/2),
            (h, w/2),
            (h/2, w/2),
        ]

        cl = self.cam_model.projectPixelTo3dRay((h/2, 0))
        cr = self.cam_model.projectPixelTo3dRay((h/2, w))
        tc = self.cam_model.projectPixelTo3dRay((0, w/2))
        bc = self.cam_model.projectPixelTo3dRay((h, w/2))
        self.theta = angle_between(cl, cr)
        self.phi = angle_between(tc, bc)
        #self.theta = 58.6/360*2*np.pi
        #self.phi = 46.6/360*2*np.pi

        #print 'theta:', self.theta
        #print 'phi:', self.phi
        #print 2*np.pi/self.theta

        draw = False
        if draw and self.T is not None:
            markers = []
            for i, pixel in enumerate(pixels):
                point = self.max_range*np.array(self.cam_model.projectPixelTo3dRay(pixel))
                p0 = self.T.dot(np.array([0, 0, 0, 1]))
                p1 = self.T.dot(np.concatenate([point, [1]]))

                m = Marker()
                m.header.frame_id = self.display_frame
                m.type = Marker.ARROW
                m.scale.x = .02
                m.scale.y = .04
                m.scale.z = 0
                m.points = [np_point_to_ros_point(p0), np_point_to_ros_point(p1)]
                m.ns = "viewcone"
                m.lifetime.secs = self.duration
                m.lifetime.nsecs = 0 # Nanoseconds
                m.id = i
                m.color = self.colors[0]
                markers.append(m)
            self.pub_clusters.publish(MarkerArray(markers))

        #for pixel in pixels:
        #    ray = self.cam_model.projectPixelTo3dRay(pixel) # Unit vector
        #    print pixel, ray, np.linalg.norm(ray) #self.cam_model.project3dToPixel(ray)
        #print

        #(0, 0) (-0.48438818291389191, -0.36310162694171244, 0.7959430235674281)
        #(480, 0) (0.26796730326906865, -0.39986398213670993, 0.8765285620950843)
        #(0, 640) (-0.43554912639556859, 0.54597003167895219, 0.7156910527626714)
        #(480, 640) (0.23618640224431237, 0.5893623308339383, 0.7725723437898068)
        #(240, 320) (-0.14803019170542384, 0.14989220669542916, 0.9775578697527989)


        #self.sub_table.unregister()
        #param_dict = rospy.get_param("/head_mount_kinect")
        #print self.camera_info.K # Intrinsic camera matrix
        # rosparam get head_mount_kinect
        #print self.camera_info

    def cb_table(self, data_msg):
        self.best_table = data_msg
        #self.sub_table.unregister()

    def cb_markers(self, data_msg):
        start_time = time()

        #self.pub_clusters.publish(data_msg)
        #return # NOTE - automatically updates so the problem is with the code then
        # Maybe if the display is relative to the head that it works more quickly
        # Changing the queue size in rviz doesn't help 

        if self.T is None or not self.best_table: return
        T = self.T.copy()
        T = np.linalg.inv(self.tf_listener.asMatrix(self.head_frame,
                Header(0,rospy.Time(0),self.display_frame)))

        #self.sub.unregister()
        #raw_input('Continue?')

        markers = [] 
        boxes = []
        results = []
        # TODO - compare detections to position on table

        # compute center of table position
        p = self.best_table.pose.position
        table = [float(q) for q in list(T*np.matrix([p.x,p.y,p.z,1.0]).T)[:3]]

        trans = trans_from_pose(ros_pose_to_or_pose(self.best_table.pose))
        hull_vertices = np.array([ros_point_to_np_point(point) for point in self.best_table.convex_hull])
        table_vertices = T.dot(trans).dot(np.vstack([hull_vertices.T, np.ones(len(hull_vertices))]))[:3,:].T
        (tx, ty, tz), (tw, th, tl), tquat = aabb_from_points(table_vertices.T)

        #print (tx, ty, tz), (tw, th, tl)
        #if abs(tz - .5) > .25: return
        #if tl > .1: return
        #print 'Found table'
        # (1.3319649845308725, 0.25374539330480506, 0.49599036616583736) (0.6641045345790979, 1.3176815663987005, 0.090323416981836324)

        for i, cluster in enumerate(data_msg.markers):
            #points = [ [float(q) for q in \
            #        list(T*np.matrix([p.x,p.y,p.z, 1.0]).T)[:3]] \
            #        for p in cluster.points]
            #points = np.array(points)

            head_points = np.array([ros_point_to_np_point(point) for point in cluster.points])
            points = T.dot(np.vstack([head_points.T, np.ones(len(head_points))]))[:3,:].T
            ros_pts = [Point(*p) for p in points]

            #x,y,z = np.mean(points, axis=0) # NOTE - should use the average instead of the mean here
            #x,y,z = (np.max(points,axis=0) + np.min(points,axis=0))/2
            #w,h,l = np.max(points,axis=0) - np.min(points,axis=0)
            #quat = np.array([0, 0, 0, 1])
            (x, y, z), (w, h, l), quat = aabb_from_points(points.T)

            volume = w*h*l
            density = len(points)/volume

            # not a tight cluster of points
            if density < self.density: continue
            #box too large
            #if w*h*l > self.max_area: continue
            
            # TODO - check if center contained within table

            if abs(z - l/2 - table[2]) > .05: continue
            if x - w/2 < tx - tw/2 or x + w/2 > tx + tw/2 or \
               y - h/2 < ty - th/2 or y + h/2 > ty + th/2: continue

            # TODO - queue size!

            t0 = time()
            #(x, y, z), (w, h, l), quat = aabb_from_points(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points_2D(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points2(points)
            #(x, y, z), (w, h, l), quat = oobb_from_points2_2D(points)
            #(x, y, z), (w, h, l), quat = oobb_from_points_2D_convex(points)
            (x, y, z), (w, h, l), quat = robust_oobb_from_points_2D_convex(points)
            print points.shape, time() - t0

            #print (x, y, z), (w, h, l)
            #print z - l/2, table[2]
            #print quat

            scale = 1 # TODO - take bounding box of the top .95 percent
            w *= scale
            h *= scale
            l *= scale

            #m = Marker()
            m = deepcopy(cluster)
            m.header = Header(0, rospy.Time(0), self.display_frame)
            m.points = ros_pts
            m.ns = "clusters"
            #markers.append(m)

            #bb = Marker()
            bb = deepcopy(m)
            bb.header.frame_id = self.display_frame
            bb.type = Marker.CUBE
            bb.pose = Pose(Point(x,y,z), Quaternion(*quat))
            bb.scale.x = w
            bb.scale.y = h
            bb.scale.z = l
            bb.points = []
            bb.ns = "bounding_boxes"
            bb.lifetime.secs = self.duration
            bb.lifetime.nsecs = 0 # Nanoseconds
            #bb.frame_locked = False
            #boxes.append(bb)
            results.append((y+h, x+w, w*h*l, m, bb))
            #print 'Object'
            #print bb
            #print

        obj_results = results[:]
        (x, y, z), (w, h, l), quat = robust_oobb_from_points_2D_convex(table_vertices)
        #(x, y, z), (w, h, l), quat = aabb_from_points(table_vertices.T)

        #table_bb = deepcopy(m)
        table_bb = Marker()
        table_bb.header.frame_id = self.display_frame
        table_bb.type = Marker.CUBE
        #table_bb.action = 0
        table_bb.pose = Pose(Point(x,y,z), Quaternion(*quat))
        table_bb.scale.x = w
        table_bb.scale.y = h
        table_bb.scale.z = l
        table_bb.points = []
        table_bb.ns = "bounding_boxes"
        table_bb.lifetime.secs = self.duration
        table_bb.lifetime.nsecs = 0 # Nanoseconds
        #boxes.append(table_bb)
        #results.append((w*h*l, x, y, None, table_bb))
        results.append((y+h, x+w, w*h*l, None, table_bb))

        #print 'Table'
        #print table_bb
        print

        # TODO - some of the clusters overlap
        #####

        results = sorted(results, \
                key=lambda x: (x[0], x[1], x[2]) , reverse=True)[:self.max_clusters]

        for i, (area,x,y,m,bb) in enumerate(results):
            if m is not None:
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
        # for i in range(len(results), self.max_clusters): # TODO - this is ineffective
        #     m = Marker()
        #     m.id = i
        #     m.action = Marker.DELETE
        #     m.ns = 'bounding_boxes'
        #     boxes.append(m)
        #     m = deepcopy(m)
        #     m.ns = 'clusters'
        #     markers.append(m)

        self.pub_clusters.publish(MarkerArray(markers))
        self.pub_bb.publish(MarkerArray(boxes))

        print 'Detections:', len(results), time() - start_time

        # TODO - workspace trajectories

        plan = False
        if plan and not self.detected and table_bb is not None and len(obj_results) != 0:
            rospy.loginfo("detection")
            self.detected = True
            print '\n'*10
            _, _, _, _, bb = obj_results[0]

            set_point(self.robot, [0, 0, 0])

            table = centered_box_body(self.env, table_bb.scale.x, table_bb.scale.y, table_bb.scale.z, name='table', color=(0, 0, 1))
            set_pose(table, ros_pose_to_or_pose(table_bb.pose))    
            self.env.Add(table)

            body = centered_box_body(self.env, bb.scale.x, bb.scale.y, bb.scale.z, name='obj', color=(1, 0, 0))
            set_pose(body, ros_pose_to_or_pose(bb.pose))
            self.env.Add(body)

            approach_config = self.robot.GetDOFValues(self.robot.GetActiveManipulator().GetArmIndices())

            grasp_approach = GRASP_APPROACHES.TOP # TOP | SIDE
            grasp_type = GRASP_TYPES.GRASP # TOUCH | GRASP | COLLIDE

            viewer = True
            if viewer:
                self.env.SetViewer('qtcoin')

            #env_file = '/home/caelan/catkin_wsd/src/lis_pr2_pkg_experimental/lis_vision/scripts/env.dae'
            #self.env.Save(env_file, Environment.SelectionOptions.Everything)
            #rospy.loginfo("Saved!")
            #return

            do_motion_planning = True
            results = None
            with body:
                with self.robot:
                    grasps = get_grasps(self.env, self.robot, body, grasp_approach, grasp_type)
                    grasps = filter(lambda g: g.grasp_trans[0,3] > 0, grasps)
                    results = feasible_pick_place(self.env, self.robot, body, grasps, approach_config, do_motion_planning)

            if results is not None:
                grasp_config, vec_traj, arm_traj = results
                rospy.loginfo("Success pick")
                self.robot.SetDOFValues(grasp_config, self.robot.GetActiveManipulator().GetArmIndices())
                #return

                move = True
                if move:
                    raw_input('Move?')

                    #arm_traj.smooth()
                    #path = list(reversed(vec_traj.waypoints()[:-1] + arm_traj.waypoints()[:-1]))
                    #path = list(reversed(vec_traj.path()[:-1] + arm_traj.path()[:-1]))
                    #self.uc.command_joint_trajectory('l', path, [4]*len(path), blocking=True)

                    #rospy.loginfo('Start viewer plan?')
                    #for conf in path:
                    #  print conf
                    #  self.robot.SetDOFValues(conf, self.robot.GetActiveManipulator().GetArmIndices())
                    #  #self.uc.command_joint_pose('l', conf, 1, blocking=True)
                    #  raw_input('Continue?')
                    #return

                    dt = .01
                    scale = 10
                    rest = 3
                    max_effort = 25 # 15 is too small, 20 works but is a little weak
                    
                    ##path = list(reversed(vec_traj.samples(time_step=dt)[:-1] + arm_traj.samples(time_step=dt)[:-1]))
                    #path = list(reversed(vec_traj.samples(time_step=dt) + arm_traj.samples(time_step=dt)))
                    #self.robot.SetDOFValues(path[-1], self.robot.GetActiveManipulator().GetArmIndices())
                    #self.uc.command_joint_trajectory(self.arm, path, [.25*dt]*len(path), blocking=True)
                    #rospy.loginfo("Executed Trajectory!")

                    arm_traj.shortcut() # shortcut, replan, smooth
                    arm_path = list(reversed(arm_traj.samples(time_step=dt)))
                    rospy.loginfo('Approach waypoints: %s, time: %s'%(len(arm_path), len(arm_path)*scale*dt))

                    #vec_traj.shortcut() # shortcut, replan, smooth
                    vec_path = list(reversed(vec_traj.samples(time_step=dt)))
                    rospy.loginfo('Approach waypoints: %s, time: %s'%(len(vec_path), len(vec_path)*scale*dt))

                    # TODO - could do no blocking then terminate if collision

                    # NOTE - times must be a cumsum
                    self.uc.command_joint_trajectory(self.arm, arm_path, np.cumsum([scale*dt]*len(arm_path)), blocking=True)
                    rospy.loginfo("Executed Approach Trajectory!")
                    rospy.sleep(rest)

                    times = np.cumsum([scale*dt]*len(vec_path))
                    self.uc.command_joint_trajectory(self.arm, vec_path, times, blocking=False)
                    event = self.uc.wait_for_gripper_event(self.arm, times[-1])
                    if event:
                        self.uc.clients["%s_joint"%self.arm].cancel() # TODO - prevent moving downwards afterwards
                    rospy.loginfo("Gripper event: %s"%event)
                    rospy.loginfo("Executed Vector Trajectory!")
                    rospy.sleep(rest)

                    #self.uc.close_gripper(self.arm, blocking=True)
                    self.uc.command_gripper(self.arm, 0, max_effort, blocking=True, timeout=2) # Max effort is 10000 (or -1). Ari's effort was 100
                    rospy.sleep(rest)

                    self.uc.command_joint_trajectory(self.arm, vec_path[::-1], np.cumsum([scale*dt]*len(vec_path)), blocking=True)
                    rospy.loginfo("Executed Vector Trajectory!")
                    rospy.sleep(rest)

                    self.uc.command_joint_trajectory(self.arm, arm_path[::-1], np.cumsum([scale*dt]*len(arm_path)), blocking=True)
                    rospy.loginfo("Executed Approach Trajectory!")
                    rospy.sleep(rest)

                    place = True
                    if place:
                        self.uc.command_joint_trajectory(self.arm, arm_path, np.cumsum([scale*dt]*len(arm_path)), blocking=True)
                        rospy.loginfo("Executed Approach Trajectory!")
                        rospy.sleep(rest)

                        self.uc.command_joint_trajectory(self.arm, vec_path, np.cumsum([scale*dt]*len(vec_path)), blocking=True)
                        rospy.loginfo("Executed Vector Trajectory!")
                        rospy.sleep(rest)
                        # TODO - how to I sense that this has collided?

                        self.uc.open_gripper(self.arm, blocking=True)
                        rospy.sleep(rest)

                        self.uc.command_joint_trajectory(self.arm, vec_path[::-1], np.cumsum([scale*dt]*len(vec_path)), blocking=True)
                        rospy.loginfo("Executed Vector Trajectory!")
                        rospy.sleep(rest)

                        self.uc.command_joint_trajectory(self.arm, arm_path[::-1], np.cumsum([scale*dt]*len(arm_path)), blocking=True)
                        rospy.loginfo("Executed Approach Trajectory!")
                        rospy.sleep(rest)

            else:
                rospy.loginfo("Failed pick")
                print 'Fail'

            #self.uc.open_gripper('l', blocking=True)
            #self.uc.wait_for_gripper_event('l')
            ##self.uc.request_gripper_event('l') # Non-blocking
            #self.uc.close_gripper('l', blocking=True)
            #self.sub.unregister()

    def updateTransform(self):
        # continuously find transform for head to baselink
        while not rospy.is_shutdown():
            try:
                #t0 = time()
                T =  self.tf_listener.asMatrix( self.head_frame,\
                        Header(0,rospy.Time(0),self.display_frame))
                self.T = np.linalg.inv(T)
                #print time() - t0 # Is pretty fast
            except:
                print "no tf"
                pass
            rospy.sleep(self.tfrom_sleep)

if __name__=="__main__":
    rospy.init_node("cluster_filters")
    tf=ClusterFilter()
    rospy.spin()




