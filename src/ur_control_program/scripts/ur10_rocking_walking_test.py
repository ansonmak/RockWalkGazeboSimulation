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

from sensor_msgs.msg import Imu
from gazebo_msgs.srv import SpawnModel
import os #print without new line
from gazebo_msgs.msg import ContactsState #contact sensor
import matplotlib.pyplot as plt #plot location map

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

  def init_spawn_service(self):
    #initialize rosservice to spawn model in gazebo
    sys.stdout.write("Waiting for gazebo spawn SDF service...")
    sys.stdout.flush()
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    self.spawn_cone = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    f = open(os.path.expanduser('~/.gazebo/models/oblique_cone/model.sdf'), 'r')
    self.cone_xml = f.read()

    print "Done"

  def subscribe_cone_imu(self):
    sys.stdout.write("Subscribing to oblique cone imu topic...")
    sys.stdout.flush()
    self.cone_imu_subscriber = rospy.Subscriber('/oblique_cone_test/imu', Imu, self.cone_imu_handler)
    print "Done"

  def cone_imu_handler(self, cone_imu_data):
    self.cone_imu = copy.deepcopy(cone_imu_data)
    # print("Angular velocity Z: %5.10f" % (self.cone_imu.angular_velocity.z))

  def subscribe_cone_bumper(self):
    sys.stdout.write("Subscribing to oblique cone bumper topic...")
    sys.stdout.flush()
    self.cone_trace = False
    self.contact_trace_x = []
    self.contact_trace_y = []
    self.cone_bumper_subscriber = rospy.Subscriber('/oblique_cone_test/bumper', ContactsState, self.cone_bumper_handler)
    print "Done"

  def cone_bumper_handler(self, cone_bumper_data):
    if self.cone_trace:
      self.contact_trace_x.append(cone_bumper_data.states[0].contact_positions[0].x)
      self.contact_trace_y.append(cone_bumper_data.states[0].contact_positions[0].y)

  def follow_target_joint(self, target_joint):
    try: 
      self.group.go(target_joint, wait=True)
    except rospy.ROSInterruptException:
      pass

  def follow_target_pose(self, target_pose):
    try:
      self.group.set_pose_target(target_pose)

      plan = self.group.plan()

      self.group.execute(plan)

      self.group.clear_pose_targets()

    except rospy.ROSInterruptException:
      pass

  def prepare_arm_pose(self):
    #set the end effector above the rod of the cone
    sys.stdout.write("Preparing arm pose...")
    sys.stdout.flush()
    #initial_joint_value = self.group.get_current_joint_values()
    # initial_joint_value[0] = 0.4500427806*pi
    # initial_joint_value[1] = -0.2735064713*pi
    # initial_joint_value[2] = 0.1704176128*pi
    # initial_joint_value[3] = -0.5256980438*pi
    # initial_joint_value[4] = -0.4813369687*pi
    # initial_joint_value[5] = -0.0467070402*pi
    initial_joint_value = [] #solution with error on getting current joint values
    initial_joint_value.append(0.4500427806*pi)
    initial_joint_value.append(-0.2735064713*pi)
    initial_joint_value.append(0.1704176128*pi)
    initial_joint_value.append(-0.5256980438*pi)
    initial_joint_value.append(-0.4813369687*pi)
    initial_joint_value.append(-0.0467070402*pi)
    self.follow_target_joint(initial_joint_value)

    # prepare_pose = geometry_msgs.msg.Pose()
    # prepare_pose.position.x = 0
    # prepare_pose.position.y = 1.0822
    # prepare_pose.position.z = 1.45
    # prepare_pose.orientation.w = 0.5332614136
    # prepare_pose.orientation.x = -0.4585729884
    # prepare_pose.orientation.y = 0.4744425516
    # prepare_pose.orientation.z = 0.5293839290
    # self.follow_target_pose(prepare_pose)

    rospy.sleep(1)

    #low down the end effector to cage the rod 
    pose = self.group.get_current_pose().pose
    pose.position.y -= 0.1
    pose.position.z -= 0.3
    self.follow_target_pose(pose)

    rospy.sleep(1)

    #pull the cone to set the rocking angle
    current_pose = self.group.get_current_pose().pose
    current_rpy = self.group.get_current_rpy()
    current_pose.position.y -= 0.6 #sim1:0.6
    current_rpy[1] -= 0 #sim1:0
    target_pose = [current_pose.position.x, current_pose.position.y, current_pose.position.z, current_rpy[0], current_rpy[1], current_rpy[2]]
    self.follow_target_pose(target_pose)

    rospy.sleep(1)
    print "Done"

  def excite_cone_rolling(self):
    #end effector moves left and right to excite the rolling motion
    sys.stdout.write("Exciting cone motion...")
    sys.stdout.flush()

    excitation_length = 0.35 #sim1:0.35
    waypoints = []

    wpose = self.group.get_current_pose().pose
    wpose.position.x += excitation_length
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= excitation_length * 2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += excitation_length
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
    self.group.execute(plan, wait=True)
    print "Done"

  def set_apex_displacement(self,displace_x,displace_y):
    #set the displacement of apex for each step

    self.displace_x = displace_x
    self.displace_y = displace_y

  def relocate_arm(self,first_step,count,dir):
    sys.stdout.write("Relocating arm position...")
    sys.stdout.flush()
    pose = geometry_msgs.msg.Pose()
    if first_step:
      pose.position.x = self.init_pose.position.x - self.displace_x
    else:
      pose.position.x = self.init_pose.position.x + dir * self.displace_x * 2
    pose.position.y = self.init_pose.position.y - self.displace_y
    pose.position.z = self.init_pose.position.z
    pose.orientation = copy.deepcopy(self.init_pose.orientation)
    self.follow_target_pose(pose)
    print "Done"

  def manipulate_apex_position_feedforward(self):
    #change the apex position to walk the cone with feedforward control

    rospy.sleep(9) #wait for cone to start

    print "Start rocking..."

    #get initial position and set displacement in x and y direction
    self.init_pose = self.group.get_current_pose().pose
    self.set_apex_displacement(0.05,0.05)

    direction = 1 #+ve: left, -ve:right
    self.relocate_arm(1, 0, direction) #first step: move to right

    for count in range(5): 
      rospy.sleep(3.5 + count*0.05) 
      self.relocate_arm(0, count, direction)
      direction *= -1

  def manipulate_apex_position_feedback(self):
    #change the apex position to walk the cone with feedback control
    rospy.sleep(9) #wait for cone to start

    print "Start rocking..."

    self.cone_trace = True

    #get initial position and set displacement in x and y direction
    self.init_pose = self.group.get_current_pose().pose
    self.set_apex_displacement(0.05,0.05)

    direction = 1 #+ve: left, -ve:right
    self.relocate_arm(1, 0, direction) #first step: move to right

    rospy.sleep(1)

    count = 0

    while count < 4:
      if abs(self.cone_imu.angular_velocity.z) < 0.15:
        self.relocate_arm(0, count, direction)
        direction *= -1
        count += 1
        while abs(self.cone_imu.angular_velocity.z) < 0.2:
          pass

    self.cone_trace = False



def main():
  ur10 = Ur10RoboticArm()
  ur10.init_spawn_service()
  ur10.subscribe_cone_imu()
  ur10.subscribe_cone_bumper()

  print "===========Press enter to start the simulation"
  raw_input()
  ur10.prepare_arm_pose()
  ur10.excite_cone_rolling()
  ur10.manipulate_apex_position_feedback()

  plt.scatter(ur10.contact_trace_x,ur10.contact_trace_y)
  plt.show()

  # cone_pose = geometry_msgs.msg.Pose()
  # cone_pose.position.x = 0
  # cone_pose.position.y = 0
  # cone_pose.position.z = 0
  # # cone_pose.orientation.w = 1
  # # cone_pose.orientation.x = 0
  # # cone_pose.orientation.y = 0
  # # cone_pose.orientation.z = 0
  # ur10.spawn_cone("oblique_cone1",ur10.cone_xml,"spawned_cone",cone_pose,"world")
  # print "Cone model spawned successfully"

  #rospy.spin()

if __name__ == '__main__':
  main()

