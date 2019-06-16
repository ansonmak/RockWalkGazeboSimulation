#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class Ur10RoboticArm(object):
  def __init__(self):
    super(Ur10RoboticArm, self).__init__()

    #init
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_control_test', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    #get info
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

def main():
  try:
    ur10 = Ur10RoboticArm()

    while not rospy.is_shutdown():
      current_pose = ur10.group.get_current_pose().pose
      current_joint = ur10.group.get_current_joint_values()
      print ("Current position :")
      print ("x: %5.10f" % (current_pose.position.x))
      print ("y: %5.10f" % (current_pose.position.y))
      print ("z: %5.10f" % (current_pose.position.z))
      print ("Current orientation :")
      print ("w: %5.10f" % (current_pose.orientation.w))
      print ("x: %5.10f" % (current_pose.orientation.x))
      print ("y: %5.10f" % (current_pose.orientation.y))
      print ("z: %5.10f" % (current_pose.orientation.z))
      print ("Current joint values:") 
      for x in range(len(current_joint)):
        print ("joint[%d]: %5.10f" % (x, current_joint[x]))
      raw_input()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

