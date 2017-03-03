#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Vector3
from object_recognition_msgs.msg import Table, TableArray
from std_msgs.msg import Header, ColorRGBA
from threading import Thread
from argparse import ArgumentParser
import tf
import numpy as np
import pdb
from copy import deepcopy

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject



def reject_outliers(data, extra_data, m=2):
    rejects = np.all(abs(data - np.mean(data)) < m * np.std(data), axis=1)
    new_data = data[rejects]
    new_extra = extra_data[rejects]
    return new_data, new_extra


get_xyz= lambda p: np.array ([ p.x,p.y,p.z])

class Tracker:
    table_frame = "head_mount_kinect_rgb_optical_frame"
    base_frame = "base_link"
    diff_thresh = 0.009
    buffer_size = 5
    obj_types = ['table', 'bb']

    colors = {'bb':ColorRGBA(51/255.,153/255.,1, .8),\
            'table': ColorRGBA(51/255., 25/255.,0,.7)}

    
    def __init__(self, debug=False, tf_listener=None):
        self.debug = debug
        self.arr= {obj_type: np.array([ [float('inf')]*3]*self.buffer_size) \
                for obj_type in self.obj_types}
        self.arr_dims= {obj_type: np.array([ [float('inf')]*3]*self.buffer_size) \
                for obj_type in self.obj_types}

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        
        # keep track of head to base link continuously
        self.T = None
        self.transform_finder = Thread(target=self.updateTransform)
        self.transform_finder.start()

        # keep track of table location
        self.best_table = None
        self.sub_table = rospy.Subscriber("/best_table", Table, self.cb_obj,callback_args="table", queue_size=1)
    
        # keep track of object
        self.best_obj = None
        self.sub_bb = rospy.Subscriber("/bounding_boxes", MarkerArray, self.cb_obj, callback_args="bb",queue_size=1)

        self.pub = rospy.Publisher("/table_and_object", MarkerArray, queue_size=2)
        self.pub_co = rospy.Publisher("/ari_collision_object", CollisionObject, queue_size=1)


        rospy.loginfo("done init")
   
    # return moving average with outlier rejection
    def get_mean(self, obj_type, pts, dim):
 
        self.arr[obj_type] = np.roll(self.arr[obj_type], -3)
        self.arr[obj_type][-1, :] = pts 
        
        self.arr_dims[obj_type] = np.roll(self.arr_dims[obj_type], -3)
        self.arr_dims[obj_type][-1, :] = dim

        if np.mean(self.arr[obj_type]) == float('inf'):
            mean_pos, mean_dim = None, None # we dont have enough data yet
        else:
            filtered_pos, filtered_dim = reject_outliers(self.arr[obj_type], self.arr_dims[obj_type])
            mean_pos = np.mean(filtered_pos, axis=0)
            mean_dim = np.mean(filtered_dim, axis=0)
        return mean_pos, mean_dim

    def pub_obj(self, obj_type, pos, dim):
        m = Marker()
        m.id = self.obj_types.index(obj_type)
        m.ns = "simple_tabletop"
        m.type = Marker.CUBE
        m.pose = Pose(Point(*pos), Quaternion(0,0,0,1))
        m.header = Header(0, rospy.Time(0), "base_link")
        m.scale = Vector3(*dim)
        m.color = self.colors[obj_type]

        mtext = deepcopy(m)
        mtext.type = Marker.TEXT_VIEW_FACING
        mtext.id += 100
        mtext.text = obj_type

        M = MarkerArray( [m, mtext])
        self.pub.publish(M)

        #moveit collision objects

        co =  CollisionObject()
        co.header = deepcopy(m.header)
        co.id = obj_type

        obj_shape = SolidPrimitive()
        obj_shape.type = SolidPrimitive.BOX
        obj_shape.dimensions = list(dim)
        co.primitives.append(obj_shape)
        
        obj_pose = Pose( Point(*pos), Quaternion(0,0,0,1) )
        co.primitive_poses= [obj_pose]
        self.pub_co.publish(co)





        

    def cb_obj(self,data_msg, obj_type):
        if self.T is None: return
        #if obj_type=="bb": return
        if self.debug: self.sub_table.unregister()
        
        if obj_type=="table":
            pose = data_msg.pose
            dim = np.array( [60, 120, 1])*1e-2  # lazy...should fix
        elif obj_type=="bb":
            if len(data_msg.markers) == 0: return
            pose = data_msg.markers[0].pose
            dim = get_xyz(data_msg.markers[0].scale)
        else:
            print "%s did not move " % obj_type
            return
        if obj_type=="table":
            pts =np.matrix(  get_xyz(pose.position).tolist() + [1] )
            pts_transformed = [float(q) for q in list(self.T*pts.T)[:3]]
        else:
            pts_transformed = get_xyz(pose.position)
        mean_pos, mean_dim= self.get_mean( obj_type, pts_transformed, dim)
        if mean_pos is None: return
        self.pub_obj(obj_type,mean_pos, mean_dim)
        #if obj_type=="bb": print mean_pos, mean_dim


    def updateTransform(self):
        # continuously find transform for head to baselink
        while not rospy.is_shutdown():
            try:
                T =  self.tf_listener.asMatrix( self.table_frame, Header(0,rospy.Time(0),self.base_frame))
                T = np.linalg.inv(T)
                self.T = T
            except:
                pass
            rospy.sleep(1)

        

if __name__=="__main__":
    rospy.init_node("tracker")
    try:
        parser = ArgumentParser()
        parser.add_argument("-d","--debug", action="store_true")
        args = parser.parse_args()
        debug = args.debug
    except:
        debug = False # being run by rosnode

    tr = Tracker(debug)
    rospy.spin()
       
  
