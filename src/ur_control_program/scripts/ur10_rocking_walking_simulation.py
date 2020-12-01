#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, Wrench, Vector3
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import math
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import SpawnModel, SetModelState, ApplyBodyWrench, BodyRequest
import os #print without new line, use ~/ for path
from gazebo_msgs.msg import ContactsState, ModelState
import xml.etree.cElementTree as ET #handle xml file
from tf.transformations import quaternion_from_euler
#plot contact map
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
#test vscode push

def print_1line(message):
  sys.stdout.write(message)
  sys.stdout.flush()

fig = plt.figure()
ax = fig.add_subplot(1,1,1)

def animate(i,xs,ys):
  ax.plot(xs,ys,color='b')
  ax.set_xlim(-1.5, 1.5)
  ax.set_ylim(0, 2.5)


class Ur10RoboticArm:
  def __init__(self, group_name):
    #init
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group = moveit_commander.MoveGroupCommander(group_name)

    #print info
    self.planning_frame = self.group.get_planning_frame()
    print "============ Reference frame: %s" % self.planning_frame

    self.eef_link = self.group.get_end_effector_link()
    print "============ End effector: %s" % self.eef_link

    self.group_names = self.robot.get_group_names()
    print "============ Robot Groups:", self.robot.get_group_names()

    print "============ Printing robot state"
    print self.robot.get_current_state()
    print ""

  def print_eef_state(self):
    print "============End Effector State:"
    print "Position:"
    cur_pose = self.group.get_current_pose().pose
    print ("x: %13.10f" % (cur_pose.position.x))
    print ("y: %13.10f" % (cur_pose.position.y))
    print ("z: %13.10f" % (cur_pose.position.z))
    print "Orientation:"
    print ("w: %13.10f" % (cur_pose.orientation.w))
    print ("x: %13.10f" % (cur_pose.orientation.x))
    print ("y: %13.10f" % (cur_pose.orientation.y))
    print ("z: %13.10f" % (cur_pose.orientation.z))
    print "Roll Pitch Yaw:"
    cur_rpy = self.group.get_current_rpy()
    for i in range(len(cur_rpy)):
      print cur_rpy[i],
    print ""

  #return two lists of position and rpy
  def get_cur_position_rpy(self):
    cur_pose = self.group.get_current_pose().pose
    cur_position = [cur_pose.position.x, cur_pose.position.y, cur_pose.position.z]
    cur_rpy = self.group.get_current_rpy()
    return cur_position, cur_rpy

  def follow_target_joint(self, target_joint):
    try: 
      self.group.go(target_joint, wait=True)
    except rospy.ROSInterruptException:
      pass

  #target_pose: list of position + orientation/RPY
  def follow_target_pose(self, target_pose):
    try:
      self.group.set_pose_target(target_pose)
      plan = self.group.plan()
      self.group.execute(plan)
      self.group.clear_pose_targets()
    except rospy.ROSInterruptException:
      pass

  #postion: list of x y z, rpy: list of row pitch yaw
  def set_position_rpy(self, position_set, rpy_set):
    self.follow_target_pose(position_set+rpy_set)

  def change_position_rpy(self, position_change, rpy_change): #postion: list of x y z, rpy: list of row pitch yaw
    current_pose = self.group.get_current_pose().pose
    current_rpy = self.group.get_current_rpy()
    current_pose.position.x += position_change[0]
    current_pose.position.y += position_change[1]
    current_pose.position.z += position_change[2]
    current_rpy[0] += rpy_change[0]
    current_rpy[1] += rpy_change[1]
    current_rpy[2] += rpy_change[2]
    target_pose = [current_pose.position.x, current_pose.position.y, current_pose.position.z, current_rpy[0], current_rpy[1], current_rpy[2]]
    self.follow_target_pose(target_pose)

  def initialize_arm_pose(self):
    print_1line("Initializing arm pose...")
    initial_joint_value = []
    initial_joint_value.append(1.4358434279)
    initial_joint_value.append(0.2310136839)
    initial_joint_value.append(-0.6234063850)
    initial_joint_value.append(-1.6639328365)
    initial_joint_value.append(-1.4917641477)
    initial_joint_value.append(-0.1808708788)
    self.follow_target_joint(initial_joint_value)
    #align x, pitch and yaw to zero
    position, rpy = self.get_cur_position_rpy()
    position[0]  = rpy[1] = rpy[2] = 0
    position[1] = 1.2
    position[2] = 1
    rpy[0] = -2.1  #-3.1:eef horizontal
    self.follow_target_pose(position+rpy)
    print "Done"



class Oblique_cone:
  def __init__(self):
    self.model_path = "~/ur_ws/src/models/oblique_cone/model.sdf"
    #geometrical parameters (unit:meter)
    self.base_radius = 0.35
    self.base_height = 0.01
    self.rod_radius = 0.009
    self.rod_y = -0.34
    self.rod_z = 0.75
    self.rod_roll = 0
    #spawn position
    self.spawn_pose = Pose()
    self.spawn_pose.position.x = 0
    self.spawn_pose.position.y = 3
    self.spawn_pose.position.z = 0
    #state info
    self.state = ModelState()
    self.state.model_name =  "oblique_cone"

  def spawn_model(self):
    #initialize rosservice to spawn model in gazebo
    print_1line("Waiting for gazebo spawn SDF service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #setup spawn function
    self.spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #get model file
    f = open(os.path.expanduser(self.model_path), 'r')
    self.cone_xml = f.read()
    #spawn model
    self.spawn("oblique_cone",self.cone_xml,"",self.spawn_pose,"world")
    print "Cone model spawned"

  def subscribe_imu(self):
    print_1line("Subscribing to oblique cone imu topic...")
    self.imu_subscriber = rospy.Subscriber('/oblique_cone/imu', Imu, self.imu_handler)
    print "Done"

  def imu_handler(self, imu_data):
    self.imu = copy.deepcopy(imu_data)
    # print("Angular velocity Z: %5.10f" % (self.imu.angular_velocity.z))

  def subscribe_bumper(self):
    print_1line("Subscribing to oblique cone bumper topic...")
    self.is_contact_trace = False
    self.contact_trace_x = []
    self.contact_trace_y = []
    self.is_estimate_roll_length = False
    self.roll_length = 0
    self.prev_contact_x = 0
    self.prev_contact_y = 0
    self.bumper_subscriber = rospy.Subscriber('/oblique_cone/bumper', ContactsState, self.bumper_handler)
    print "Done"

  def reset_contact_trace(self):
    self.is_contact_trace = False
    self.contact_trace_x[:] = []
    self.contact_trace_y[:] = []

  def reset_roll_length(self):
    self.is_estimate_roll_length = False
    self.roll_length = 0
    self.prev_contact_x = 0
    self.prev_contact_y = 0

  def bumper_handler(self, bumper_data):
    if self.is_contact_trace:
      contact_x = bumper_data.states[0].contact_positions[0].x
      contact_y = bumper_data.states[0].contact_positions[0].y
      self.contact_trace_x.append(contact_x)
      self.contact_trace_y.append(contact_y)
    
    if self.is_estimate_roll_length:
      contact_x = bumper_data.states[0].contact_positions[0].x
      contact_y = bumper_data.states[0].contact_positions[0].y
      if self.prev_contact_x != 0 and self.prev_contact_y != 0:
        self.roll_length += math.hypot(contact_x - self.prev_contact_x, contact_y - self.prev_contact_y)
      self.prev_contact_x = contact_x
      self.prev_contact_y = contact_y

  def subscribe_sensors_topic(self):
    self.subscribe_imu()
    self.subscribe_bumper()

  def init_set_state_service(self):
    print_1line("Waiting for gazebo set model state service...")
    rospy.wait_for_service('/gazebo/set_model_state')
    #setup set state function
    self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    print "Done"

  def set_pose(self, target_pose):
    self.state.pose = target_pose
    self.set_state(self.state)

  def reset(self):
    self.set_pose(self.spawn_pose)

  def init_wrench_service(self):
    print_1line("Waiting for gazebo body wrench service...")
    #setup apply wrench service
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    #setup clear wrench service
    rospy.wait_for_service('/gazebo/clear_body_wrenches')
    self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
    print "Done"

  def get_roll_angle(self):
    return math.degrees(self.roll_length/self.base_radius)

  def excite_cone_motion(self, excitation_angle):
    print "Exciting cone motion..."
    excitation_force = 8
    wrench = Wrench()
    wrench.force.x = excitation_force
    self.apply_wrench("oblique_cone::cone", "world", Point(), wrench, rospy.Time(), rospy.Duration(-1))
    self.is_estimate_roll_length = True

    check_rate = rospy.Rate(20)
    while self.get_roll_angle() < excitation_angle:
      print ("Rolling Angle: %.4f" % (self.get_roll_angle()))
      check_rate.sleep()

    self.reset_roll_length()
    wrench.force.x = -1 * excitation_force
    self.apply_wrench("oblique_cone::cone", "world", Point(), wrench, rospy.Time(), rospy.Duration(-1))
    self.clear_wrench("oblique_cone::cone")
    print "Excitation Done"

  def plot_contact_trace(self):
    self.is_contact_trace = True
    ani = animation.FuncAnimation(fig, animate, fargs=(self.contact_trace_x, self.contact_trace_y), interval=100)
    plt.show()



class Rocking_Walking_Manipulation:
  def __init__(self):
    #manipulation parameters
    self.h_eef = 1
    self.roll_eef = -2.4
    self.length_ab = 1.1 #meter
    self.angle_xb = 10 #degree
    self.step = 8
    self.displace_x = 0.03
    self.displace_y = 0.08

  def print_setting(self):
    print "============Manipulation Parameters:"
    print ("EEF roll: %.2f" % (self.roll_eef))
    print ("EEF heigth: %.2f" % (self.h_eef))
    print ("Length AB: %.2f" % (self.length_ab))
    print ("Walking Angle XB: %.1f" % (self.angle_xb))
    print ("Walking Steps: %d" % (self.step))
    print ("Apex displacement X: %.3f" % (self.displace_x))
    print ("Apex displacement Y: %.3f" % (self.displace_y))

  def calculate_cone_pose(self, ur10, cone):
    ur10_pose = ur10.group.get_current_pose().pose
    target_pose = Pose()
    r = math.pi - math.asin(self.h_eef/self.length_ab) - (math.pi/2 + cone.rod_roll)
    q = quaternion_from_euler(r,0,0)
    target_pose.position.y = 0.015+ur10_pose.position.y + math.sqrt(self.length_ab*self.length_ab - self.h_eef*self.h_eef) + cone.base_radius*math.cos(r)
    target_pose.position.z = cone.base_radius * math.sin(r)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    print "============Cone Set State:"
    print("Roll angle: %.4f" % (r))
    print("Y: %.4f" % (target_pose.position.y))
    print("Z: %.4f" % (target_pose.position.z))
    return target_pose

  def relocate_arm(self, ur10, first_step, step):
    print_1line("Relocating arm position...")
    relocate_pose = Pose()
    if first_step:
      relocate_pose.position.x = self.init_pose.position.x - self.displace_x
    else:
      relocate_pose.position.x = self.init_pose.position.x + self.walking_sign * self.displace_x * 2
    relocate_pose.position.y = self.init_pose.position.y - self.displace_y * step
    relocate_pose.position.z = self.init_pose.position.z
    relocate_pose.orientation = copy.deepcopy(self.init_pose.orientation)
    ur10.follow_target_pose(relocate_pose)
    print "Done"

  def manipulate_apex_position(self, ur10, cone):
    self.init_pose = ur10.group.get_current_pose().pose
    self.walking_sign = -1 #1: move left, -1: move right
    first_step = 1 #first step move right
    step = 1

    # while abs(cone.imu.angular_velocity.z) < 0.3:
    #   pass
    rospy.sleep(5)
    print "Start rocking..."
    cone.is_contact_trace = True #start to trace the contact point

    while step <= self.step and not rospy.is_shutdown():
      if abs(cone.imu.angular_velocity.z) < 0.2:
        self.relocate_arm(ur10, first_step, step)
        first_step = 0
        step += 1
        self.walking_sign *= -1
        while abs(cone.imu.angular_velocity.z) < 0.3:
          pass

    print "============Close the figure window to continue"
    plt.scatter(cone.contact_trace_x,cone.contact_trace_y)
    plt.show()
    cone.reset_contact_trace()



def main():
  rospy.init_node('rocking_walking_simulation', anonymous=True)
  #create ur10 object
  ur10 = Ur10RoboticArm("manipulator")
  ur10.initialize_arm_pose()
  #create cone object
  cone = Oblique_cone()
  cone.spawn_model()
  cone.subscribe_sensors_topic()
  cone.init_set_state_service()
  cone.reset() #reset cone position
  cone.init_wrench_service()
  #create manipulation object
  rwm = Rocking_Walking_Manipulation()

  is_set = False

  while not rospy.is_shutdown():

    while not is_set:
      ur10.print_eef_state()
      #eef pose setting
      print "============Adjust end effector pose (Press enter for default value)"
      position,rpy = ur10.get_cur_position_rpy()
      position[1] = float(raw_input("EEF y position: ") or position[1])
      rwm.h_eef = position[2] = float(raw_input("EEF height: ") or rwm.h_eef)
      rwm.roll_eef = rpy[0] = float(raw_input("EEF roll: ") or rpy[0])
      print_1line("Adjusting End effector...")
      ur10.follow_target_pose(position+rpy)
      print "Done"
      #manipulation setting
      rwm.print_setting()
      print "============Manipulation Setting (Press enter for default value)"
      rwm.length_ab = float(raw_input("Length AB (meter): ") or rwm.length_ab)
      rwm.angle_xb = float(raw_input ("Walking Angle XB (angle): ") or rwm.angle_xb)
      rwm.step = int(raw_input("Walking Steps: ") or rwm.step)
      rwm.displace_x = float(raw_input("Apex displacement X: ") or rwm.displace_x)
      rwm.displace_y = float(raw_input("Apex displacement Y: ") or rwm.displace_y)
      cone.set_pose(rwm.calculate_cone_pose(ur10,cone))

      confirm_setting = str(raw_input("Confirm? [y/n]")).lower().strip()
      print ""
      if confirm_setting == 'y':
        is_set = True
      elif confirm_setting =='n':
        cone.reset()
        continue

    print "============Press ENTER to start"
    raw_input()

    #Exciting Cone motion by applying wrench
    cone.excite_cone_motion(rwm.angle_xb)
    #manipulation
    rwm.manipulate_apex_position(ur10,cone)

    print "============Manipulation Ended"
    change_setting = str(raw_input("Adjust Setting? [y/n]")).lower().strip()
    cone.reset()
    ur10.initialize_arm_pose()
    ur10.follow_target_pose(rwm.init_pose)
    print ""
    if change_setting == 'y':
      is_set = False
    elif change_setting =='n':
      cone.set_pose(rwm.calculate_cone_pose(ur10,cone))


  rospy.spin()

if __name__ == '__main__':
  main()

