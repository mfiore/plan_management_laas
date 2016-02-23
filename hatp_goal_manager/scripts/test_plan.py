#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import shary3_msgs.msg

def testPlan():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('shary3/manage_goal', shary3_msgs.msg.ManageGoalAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal="AssembleAllBrackets"
    parameters=[]
    goal_msg = shary3_msgs.msg.ManageGoalGoal(goal,parameters)

    # Sends the goal to the action server.
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_plan')
        result = testPlan()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"