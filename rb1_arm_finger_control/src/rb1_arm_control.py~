#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def rb1_move_group_control():
  moveit_commander.roscpp_initialize(sys.argv)

  rospy.init_node('rb1_move_group_control',anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("rb1_arm")
  display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
  
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

# Plan a joint angle value
  group_variable_values[3] = 2
  group_variable_values[4] = 2
  group_variable_values[5] = 1.5
  #group_variable_values[9] = 1
  group.set_joint_value_target(group_variable_values)
  # Joint planing
  plan2 = group.plan()

  #print "============ Waiting while RVIZ displays plan2..."
  #rospy.sleep(5)

  # Uncomment below line when working with a real robot
  group.go(wait=True)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()


if __name__=='__main__':
  try:
    rb1_move_group_control()
  except rospy.ROSInterruptException:
    pass
