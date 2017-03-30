import sys

from openravepy import Environment, DebugLevel, RaveSetDebugLevel
from openravepy.misc import SetViewerUserThread, DrawAxes

from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from pick_and_place import feasible_pick_place, get_grasps, initialize_openrave
from time import time
from random import shuffle

# https://github.com/jsk-ros-pkg/openrave_planning
# https://hub.docker.com/r/personalrobotics/ros-openrave/
# https://github.com/personalrobotics

def script(env, arm='l'):
  min_delta = .02

  initialize_openrave(env, arm, min_delta=min_delta)

  robot = env.GetRobots()[0]
  body = filter(lambda b: not b.IsRobot(), env.GetBodies())[0]
  print robot, body

  approach_config = robot.GetDOFValues(robot.GetActiveManipulator().GetArmIndices())

  grasp_approach = GRASP_APPROACHES.TOP # TOP | SIDE
  grasp_type = GRASP_TYPES.TOUCH # TOUCH | GRASP | COLLIDE
  do_motion_planning = False

  # NOTE - the top grasps don't work with the vector trajectory
  # TODO - the model for the object geometry isn't right anyways...

  # TODO - make a bounding box of the table
  # TODO - the object is too wide

  t0 = time()
  with body:
      with robot:
          grasps = get_grasps(env, robot, body, grasp_approach, grasp_type)
          grasps = filter(lambda g: g.grasp_trans[0,3] > 0, grasps)
          shuffle(grasps)
          print grasps
          results = feasible_pick_place(env, robot, body, grasps, approach_config, do_motion_planning)
  print time() - t0

  if results is not None:
      grasp_config, vec_traj, arm_traj = results
      print 'Success!'

      if vec_traj is not None:
        #arm_traj.smooth()
        #print approach_config
        #print (vec_traj.waypoints() + arm_traj.waypoints())[::-1]
        #path = reversed(vec_traj.waypoints()[:-1] + arm_traj.waypoints()[:-1])
        path = reversed(vec_traj.path()[:-1] + arm_traj.path()[:-1])
        for conf in path:
          print conf
          robot.SetDOFValues(conf, robot.GetActiveManipulator().GetArmIndices())
          raw_input('Continue?')

        #GetDuration, GetNumWaypoints, Controller
      else:
        robot.SetDOFValues(grasp_config, robot.GetActiveManipulator().GetArmIndices())
  else:
      print 'Fail'

  #DrawAxes(env, body.GetName()) # Bug in OpenRAVE
  # DrawIkparam
  # MultiManipIKSolver


  raw_input('Done!')

HELP_MESSAGE = 'python %s -p <problem>'%(__file__)

# CheckCollisionRays

def script2(env):
  raw_input('Done!')

def main():
  env = Environment()
  env.Load('env.dae')

  try:
    SetViewerUserThread(env, 'qtcoin', lambda: script2(env))
  finally:
    env.Destroy()

if __name__ == '__main__':
  main()
