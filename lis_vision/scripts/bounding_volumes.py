import heapq
import numpy as np

from tf.transformations import quaternion_from_matrix, quaternion_matrix

def ros_point_to_np_point(ros_point):
  return np.array([ros_point.x, ros_point.y, ros_point.z])

def np_point_to_ros_point(or_point):
  return Point(*or_point[:3])

def ros_quat_to_or_quat(ros_quat):
  return np.array([ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z])

def ros_pose_to_or_pose(ros_pose):
  return np.concatenate([
    ros_quat_to_or_quat(ros_pose.orientation),
    ros_point_to_np_point(ros_pose.position), 
  ])

def or_quat_to_ros_quat(or_quat): # TODO - make this return a quat object
  return np.concatenate([or_quat[1:], or_quat[:1]])

########################################

def trans_from_pose(pose):
	quat = np.concatenate([pose[1:4], pose[:1]])
	#quat = pose[:4]
	trans = quaternion_matrix(quat)
	trans[:3,3] = pose[4:]
	return trans

########################################

def rot_2D(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])

def robust_max(points, p=.95):
    #return np.max(points, axis=1)
    n = int((1-p)*points.shape[1])+1
    return np.array([[heapq.nlargest(n, points[i,:].tolist())[-1]] for i in range(points.shape[0])])

def robust_min(points, p=.95):
    #return np.min(points, axis=1)
    n = int((1-p)*points.shape[1])+1
    return np.array([[heapq.nsmallest(n, points[i,:].tolist())[-1]] for i in range(points.shape[0])]) # Removed points[i,:].tolist()[0]

def aabb_from_points(points):
  lower = np.min(points, axis=1)
  upper = np.max(points, axis=1)
  center = (upper + lower)/2
  extents = upper - lower
  trans = np.array([0, 0, 0, 1])
  return center, extents, trans

def robust_aabb_from_points(points, p=.95):
  lower = robust_min(points, p=p)
  upper = robust_max(points, p=p)
  center = (upper + lower)/2
  extents = upper - lower
  trans = np.array([0, 0, 0, 1])
  return center, extents, trans

def test_contains((cen1, ext1), (cen2, ext2)):
	return np.all(cen1+ext1/2. >= cen2+ext2/2.) and np.all(cen2-ext2/2. >= cen1-ext1/2.)

def test_contains_2D((cen1, ext1), (cen2, ext2)):
	return np.all((cen1+ext1/2. >= cen2+ext2/2.)[:2]) and np.all((cen2-ext2/2. >= cen1-ext1/2.)[:2])

def test_overlap((cen1, ext1), (cen2, ext2)):
	return np.all(cen1-ext1/2. <= cen2+ext2/2.) and np.all(cen2-ext2/2. <= cen1+ext1/2.)

def get_area((cen, ext)):
	return np.prod(ext[:2])

def get_volume((cen, ext)):
	return np.prod(ext)
