#! /usr/bin/env python

import rospy
import sys
import actionlib
import kinova_msgs.msg



""" Global variable """
arm_joint_number = 0
finger_number = 0
#prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]
finger_final_value = [0.0,0.0,0.0]


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


def unitParser(unit_, finger_value_, relative_):
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


if __name__ == '__main__':
     
    global finger_final_value
    rospy.init_node('rb1_finger_control')
    robot_type = rospy.get_param('/rb1_finger_control/robot_type')
    finger_unit = rospy.get_param('/rb1_finger_control/model')
    finger1_final_value = rospy.get_param('/rb1_finger_control/finger1_final_value')
    finger2_final_value = rospy.get_param('/rb1_finger_control/finger2_final_value')
    finger3_final_value = rospy.get_param('/rb1_finger_control/finger3_final_value')
    finger_final_relative = rospy.get_param('/rb1_finger_control/final_relative')
    kinova_robotTypeParser(robot_type)

    finger_final_value[0] = finger1_final_value
    finger_final_value[1] = finger2_final_value
    finger_final_value[2] = finger3_final_value

    if len(finger_final_value) != finger_number:
        print('Number of input values {} is not equal to number of fingers {}. Please run help to check number of fingers with different robot type.'.format(len(finger_final_value), finger_number))
        sys.exit(0)

    # get Current finger position if relative position
    getCurrentFingerPosition()

    finger_turn, finger_meter, finger_percent = unitParser(finger_unit, finger_final_value, finger_final_relative)

    try:

        if finger_number == 0:
            print('Finger number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            exit()
        else:
            positions_temp1 = [max(0.0, n) for n in finger_turn]
            positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
            positions = [float(n) for n in positions_temp2]

        print('Sending finger position ...')
        result = gripper_client(positions)
        print('Finger position sent!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')


