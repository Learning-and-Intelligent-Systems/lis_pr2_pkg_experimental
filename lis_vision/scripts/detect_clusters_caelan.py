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

def ros_point_to_or_point(ros_point):
  return np.array([ros_point.x, ros_point.y, ros_point.z])

def ros_quat_to_or_quat(ros_quat):
  return np.array([ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z])

def ros_pose_to_or_pose(ros_pose):
  return np.concatenate([
    ros_quat_to_or_quat(ros_pose.orientation),
    ros_point_to_or_point(ros_pose.position), 
  ])

def or_quat_to_ros_quat(or_quat): # TODO - make this return a quat object
  return np.concatenate([or_quat[1:], or_quat[:1]])

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

def rot_2D(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])

import heapq

def robust_max(points, p=.99):
    #return np.max(rot_vertices, axis=1)
    n = int((1-p)*points.shape[1])
    return np.array([[heapq.nlargest(n, points[i,:].tolist()[0])[-1]] for i in range(points.shape[0])])

def robust_min(points, p=.99):
    #return np.min(rot_vertices, axis=1)
    n = int((1-p)*points.shape[1])
    return np.array([[heapq.nsmallest(n, points[i,:].tolist()[0])[-1]] for i in range(points.shape[0])])

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

def aabb_from_points(points):
  center = (np.max(points, axis=1) + np.min(points, axis=1))/2
  extents = np.max(points, axis=1) - np.min(points, axis=1)
  trans = np.array([0, 0, 0, 1])
  return center, extents, trans

# env = Environment()
# #robot = env.ReadRobotXMLFile('robots/pr2-beta-sim.robot.xml') # ReadRobotData
# robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
# env.Add(robot)
# initialize_openrave(env)
# quit()

LEFT_TOP_SIDE = [1.5578610251557174, -0.13460793687920436, 1.6131468003997858, -1.473015195165653, 95.71201614935508, -1.4192964178436847, 37.862692851419574]
RIGHT_TOP_SIDE = [-1.5822931843358639, -0.08986685225797848, -1.4494837934373954, -1.4676586788444625, 29.969636385648396, -1.6421113802057423, -75.69328304787416]

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

    min_delta = .05
    arm = 'r'

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

        print self.arm, self.uc.get_joint_positions(self.arm)
        print self.other_arm, self.uc.get_joint_positions(self.other_arm)

        dt = 4.0
        #self.uc.look_down_center(blocking=False)
        self.uc.command_head([0, np.pi/3.], dt, blocking=False)

        self.uc.open_gripper(self.arm, blocking=False)
        self.uc.close_gripper(self.other_arm, blocking=False)
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

        markers = [] 
        boxes = []
        results = []
        table_vertices = None # TODO - compare detections to position on table

        # compute center of table position
        if not self.best_table:
            table = (0,0,0)
        else:
            p = self.best_table.pose.position
            table = [float(q) for q in list(self.T*np.matrix([p.x,p.y,p.z,1.0]).T)[:3]]

            trans = trans_from_pose(ros_pose_to_or_pose(self.best_table.pose))
            hull_vertices = np.array([ros_point_to_or_point(point) for point in self.best_table.convex_hull])
            table_vertices = self.T.dot(trans).dot(np.vstack([hull_vertices.T, np.ones(len(hull_vertices))]))[:3,:].T
            
        for i, cluster in enumerate(data_msg.markers):
            points =[ [float(q) for q in \
                    list(self.T*np.matrix([p.x,p.y,p.z, 1.0]).T)[:3]] \
                    for p in cluster.points]

            ros_pts = [Point(*p) for p in points]
            points = np.array(points)

            #x,y,z = np.mean(points, axis=0) # NOTE - should use the average instead of the mean here
            #x,y,z = (np.max(points,axis=0) + np.min(points,axis=0))/2
            #w,h,l = np.max(points,axis=0) - np.min(points,axis=0)
            #quat = np.array([0, 0, 0, 1])
            (x, y, z), (w, h, l), quat = aabb_from_points(points.T)

            area = w*h*l
            density = len(points)/area

            # not a tight cluster of points
            if density < self.density: continue
            #box too large
            #if w*h*l > self.max_area: continue
            
            # TODO - check if center contained within table

            if abs(z - l/2 - table[2]) > .05: continue


            t0 = time()
            #(x, y, z), (w, h, l), quat = aabb_from_points(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points_2D(points.T)
            #(x, y, z), (w, h, l), quat = oobb_from_points2(points)
            #(x, y, z), (w, h, l), quat = oobb_from_points2_2D(points)
            #(x, y, z), (w, h, l), quat = oobb_from_points_2D_convex(points)
            (x, y, z), (w, h, l), quat = robust_oobb_from_points_2D_convex(points)

            print points.shape, time() - t0
            print (x, y, z), (w, h, l)
            print z - l/2, table[2]
            print 
            print
            #print quat

            scale = 1 # TODO - take bounding box of the top .95 percent
            w *= scale
            h *= scale
            l *= scale

            m = deepcopy(cluster)
            m.header = Header(0, rospy.Time(0), "base_link")
            m.points = ros_pts
            m.ns = "clusters"
            #markers.append(m)

            bb = deepcopy(m)
            bb.type = Marker.CUBE
            bb.pose = Pose(Point(x,y,z), Quaternion(*quat))
            bb.scale.x = w
            bb.scale.y = h
            bb.scale.z = l
            bb.points = []
            bb.ns = "bounding_boxes"
            #boxes.append(bb)
            results.append((w*h*l, x, y,  m, bb))

        obj_results = results[:]
        table_bb = None
        if table_vertices is not None:
            (x, y, z), (w, h, l), quat = aabb_from_points(table_vertices.T)
            #table_bb = deepcopy(m)
            table_bb = Marker()
            table_bb.header.frame_id = 'base_link'
            table_bb.type = Marker.CUBE
            #table_bb.action = 0
            table_bb.pose = Pose(Point(x,y,z), Quaternion(*quat))
            table_bb.scale.x = w
            table_bb.scale.y = h
            table_bb.scale.z = l
            table_bb.points = []
            table_bb.ns = "bounding_boxes"
            table_bb.lifetime.nsecs = 5
            #boxes.append(table_bb)
            results.append((w*h*l, x, y, None, table_bb))
            print 'Table'
            print table_bb

        #####

        results = sorted(results, \
                key=lambda x: (x[0], x[1], x[2]) , reverse=True)

        for i, (area,x,y,m,bb) in enumerate(results):
            if m is not None:
                m.id = i
                m.color = self.colors[i%len(self.colors)]
                markers.append(m)
            bb.id = i
            bb.color = self.colors[i%len(self.colors)]
            bbtext = deepcopy(bb)
            bbtext.id = i+100
            bbtext.type = Marker.TEXT_VIEW_FACING
            bbtext.text = "%d" % i
            bbtext.color = ColorRGBA(1,1,1,1)
            boxes.append(bb)
            boxes.append(bbtext)


        self.pub_clusters.publish(MarkerArray(markers))
        self.pub_bb.publish(MarkerArray(boxes))

        # TODO - workspace trajectories

        plan = True
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
                    path = list(reversed(vec_traj.waypoints()[:-1] + arm_traj.waypoints()[:-1]))
                    #path = list(reversed(vec_traj.path()[:-1] + arm_traj.path()[:-1]))
                    #self.uc.command_joint_trajectory('l', path, [4]*len(path), blocking=True)

                    #for conf in path:
                    #  print conf
                    #  self.robot.SetDOFValues(conf, self.robot.GetActiveManipulator().GetArmIndices())
                    #  self.uc.command_joint_pose('l', conf, 1, blocking=True)
                    #  #raw_input('Continue?')

                    dt = .02
                    
                    ##path = list(reversed(vec_traj.samples(time_step=dt)[:-1] + arm_traj.samples(time_step=dt)[:-1]))
                    #path = list(reversed(vec_traj.samples(time_step=dt) + arm_traj.samples(time_step=dt)))
                    #self.robot.SetDOFValues(path[-1], self.robot.GetActiveManipulator().GetArmIndices())
                    #self.uc.command_joint_trajectory(self.arm, path, [.25*dt]*len(path), blocking=True)
                    #rospy.loginfo("Executed Trajectory!")

                    arm_traj.shortcut() # shortcut, replan, smooth
                    arm_path = list(reversed(arm_traj.samples(time_step=dt)))
                    rospy.loginfo('Approach waypoints: %s'%len(arm_path))

                    #vec_traj.shortcut() # shortcut, replan, smooth
                    vec_path = list(reversed(vec_traj.samples(time_step=dt)))
                    rospy.loginfo('Approach waypoints: %s'%len(vec_path))

                    self.uc.command_joint_trajectory(self.arm, arm_path, [4*dt]*len(arm_path), blocking=True)
                    rospy.loginfo("Executed Approach Trajectory!")
                    rospy.sleep(3)

                    self.uc.command_joint_trajectory(self.arm, vec_path, [4*dt]*len(vec_path), blocking=True)
                    rospy.loginfo("Executed Vector Trajectory!")
                    rospy.sleep(3)

                    self.uc.close_gripper(self.arm, blocking=True)
                    rospy.sleep(3)

                    self.uc.command_joint_trajectory(self.arm, vec_path[::-1], [4*dt]*len(vec_path), blocking=True)
                    rospy.loginfo("Executed Vector Trajectory!")
                    rospy.sleep(3)

                    self.uc.command_joint_trajectory(self.arm, arm_path[::-1], [4*dt]*len(arm_path), blocking=True)
                    rospy.loginfo("Executed Approach Trajectory!")
                    rospy.sleep(3)

                    place = False
                    if place:
                        self.uc.command_joint_trajectory(self.arm, arm_path, [3*dt]*len(arm_path), blocking=True)
                        rospy.loginfo("Executed Approach Trajectory!")
                        rospy.sleep(3)

                        self.uc.command_joint_trajectory(self.arm, vec_path, [dt]*len(vec_path), blocking=True)
                        rospy.loginfo("Executed Vector Trajectory!")
                        rospy.sleep(3)

                        self.uc.open_gripper(self.arm, blocking=True)
                        rospy.sleep(3)

                        self.uc.command_joint_trajectory(self.arm, vec_path[::-1], [dt]*len(vec_path), blocking=True)
                        rospy.loginfo("Executed Vector Trajectory!")
                        rospy.sleep(3)

                        self.uc.command_joint_trajectory(self.arm, arm_path[::-1], [3*dt]*len(arm_path), blocking=True)
                        rospy.loginfo("Executed Approach Trajectory!")
                        rospy.sleep(3)

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




