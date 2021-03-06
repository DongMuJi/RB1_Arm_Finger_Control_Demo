#! /usr/bin/env python

import rospy
import sys
import math
import actionlib
import kinova_msgs.msg



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


def getcurrentJointCommand(prefix_):
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
