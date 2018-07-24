#! /usr/bin/env python

import rospy
import sys
import math
import actionlib
import kinova_msgs.msg


""" Global variable """
arm_joint_number = 0
finger_number = 0
#prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]
currentJointCommand = []
finger_final_value = [0.0, 0.0, 0.0]
arm_final_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = 'arm_driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


def getcurrentJointCommand():
    # wait to get current position
    topic_address = 'arm_driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'position listener obtained message for joint position. '


def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])



def arm_unitParser(unit, joint_value, relative_):
    """ Argument unit """
    global currentJointCommand

    if unit == 'degree':
        joint_degree_command = joint_value
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    elif unit == 'radian':
        joint_degree_command = list(map(math.degrees, joint_value))
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_
        joint_radian = list(map(math.radians, joint_degree_absolute_))
    else:
        raise Exception("Joint value have to be in degree, or radian")

    return joint_degree, joint_radian


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/arm_driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None


def getCurrentFingerPosition():
    # wait to get current position
    topic_address = '/arm_driver/out/finger_position'
    rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
    print 'obtained current finger position '


def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3

def finger_unitParser(unit_, finger_value_, relative_):
    """ Argument unit """
    global currentFingerPosition

    # transform between units
    if unit_ == 'turn':
        # get absolute value
        if relative_:
            finger_turn_absolute_ = [finger_value_[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_value_

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]

    elif unit_ == 'mm':
        # get absolute value
        finger_turn_command = [x/1000 * finger_maxTurn / finger_maxDist for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    elif unit_ == 'percent':
        # get absolute value
        finger_turn_command = [x/100.0 * finger_maxTurn for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in
                                     range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_




def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    #prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger




if __name__ == '__main__':
     
    #global finger_final_value,arm_final_value
    arm_count = 0
    finger_count = 0
    rospy.init_node('rb1_arm_finger_control')
    currentJointCommand = [0]*6
    robot_type = rospy.get_param('/rb1_arm_finger_control/robot_type')

    finger_unit = rospy.get_param('/rb1_arm_finger_control/finger_unit')
    finger1_final_value = rospy.get_param('/rb1_arm_finger_control/finger1_final_value')
    finger2_final_value = rospy.get_param('/rb1_arm_finger_control/finger2_final_value')
    finger3_final_value = rospy.get_param('/rb1_arm_finger_control/finger3_final_value')
    finger_final_relative = rospy.get_param('/rb1_arm_finger_control/finger_final_relative')

    finger_final_value[0] = finger1_final_value
    finger_final_value[1] = finger2_final_value
    finger_final_value[2] = finger3_final_value

    arm_unit = rospy.get_param('/rb1_arm_finger_control/arm_unit')
    arm1_final_value = rospy.get_param('/rb1_arm_finger_control/arm1_final_value')
    arm2_final_value = rospy.get_param('/rb1_arm_finger_control/arm2_final_value')
    arm3_final_value = rospy.get_param('/rb1_arm_finger_control/arm3_final_value')
    arm4_final_value = rospy.get_param('/rb1_arm_finger_control/arm4_final_value')
    arm5_final_value = rospy.get_param('/rb1_arm_finger_control/arm5_final_value')
    arm6_final_value = rospy.get_param('/rb1_arm_finger_control/arm6_final_value')
    arm_final_relative = rospy.get_param('/rb1_arm_finger_control/arm_final_relative')

    arm_final_value[0] = arm1_final_value
    arm_final_value[1] = arm2_final_value
    arm_final_value[2] = arm3_final_value
    arm_final_value[3] = arm4_final_value
    arm_final_value[4] = arm5_final_value
    arm_final_value[5] = arm6_final_value

    kinova_robotTypeParser(robot_type)
while True:
        if len(arm_final_value) != arm_joint_number:
            print('Number of input values {} is not equal to number of joints {}. Please run help to check number of joints with different robot type.'.format(len(args.joint_value), arm_joint_number))
            sys.exit(0)

        if len(finger_final_value) != finger_number:
            print('Number of input values {} is not equal to number of fingers {}. Please run help to check number of fingers with different robot type.'.format(len(finger_final_value), finger_number))
            sys.exit(0)

    # get Current finger position if relative position
        getcurrentJointCommand()
        if arm_count == 0:
            joint_degree, joint_radian = arm_unitParser(arm_unit, arm_final_value, arm_final_relative)
            arm_count = 1
        elif arm_count == 1:
            arm_final_value = [0.0, 0.0, 0.0, 0.0, -40, 0.0]
            joint_degree, joint_radian = arm_unitParser(arm_unit, arm_final_value, arm_final_relative)
            arm_count = 2
        elif arm_count == 2:
            arm_final_value = [0.0, 0.0, 0.0, 0.0, 40, 0.0]
            joint_degree, joint_radian = arm_unitParser(arm_unit, arm_final_value, arm_final_relative)
            arm_count = 1

    # get Current finger position if relative position
        getCurrentFingerPosition()
        if finger_count == 0:
            finger_turn, finger_meter, finger_percent = finger_unitParser(finger_unit, finger_final_value, finger_final_relative)
            finger_count = 1
        elif finger_count == 1:
            finger_final_value = [0.0, 0.0, 0.0]
            finger_turn, finger_meter, finger_percent = finger_unitParser(finger_unit, finger_final_value, finger_final_relative)
            finger_count =2
        elif finger_count == 2:
            finger_final_value = [100, 100, 100]
            finger_turn, finger_meter, finger_percent = finger_unitParser(finger_unit, finger_final_value, finger_final_relative)
            finger_count = 1       



        try:

            if finger_number == 0:
                print('Finger number is 0, check with "-h" to see how to use this node.')
                finger_positions = []  # Get rid of static analysis warning that doesn't see the exit()
                exit()
            else:
                finger_positions_temp1 = [max(0.0, n) for n in finger_turn]
                finger_positions_temp2 = [min(n, finger_maxTurn) for n in finger_positions_temp1]
                finger_positions = [float(n) for n in finger_positions_temp2]

            if arm_joint_number == 0:
                print('Joint number is 0, check with "-h" to see how to use this node.')
                arm_positions = []  # Get rid of static analysis warning that doesn't see the exit()
                sys.exit()
            elif arm_joint_number == 4:
                arm_positions = [float(n) for n in joint_degree]
                arm_positions.extend([0,0])
            elif arm_joint_number == 6:
                arm_positions = [float(n) for n in joint_degree]
            else:
                sys.exit()

            result_arm = joint_angle_client(arm_positions)   
            result_finger = gripper_client(finger_positions)
            
        
 
        except rospy.ROSInterruptException:
            print('program interrupted before completion')

