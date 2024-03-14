#!/usr/bin/env python3

"""
.. module:: ResearchTrack_Assignment2

:platform: Unix
:synopsis: Python client for ResearchTrack_Assignment2

.. moduleauthor:: Ramona Ferrari S4944096@studenti.unige.it

This node statisfies the request: it implements an action client, allowing the user to set a t set a target (x,y) or cancel it. It shows on terminal also when the target is reached, printing "Target reached!"

Setting the target: the node requests both coordinates, otherwise on terminal the sentence "please provide both coordinates!" appears and the robot does not move.

The feedback of the action server is used to know when the target has been reached and it also publishes the robot position and velocity as a custom message (x,y, vel_x and vel_y),by relying on the values published on the topic /odom.

In this node it's possible to find several functions:
    -*publish_msg* ;
    -*feedback_callback*;
    -*start_client* .
    
To run the code: ``roslaunch assignment_2_2023 assignment1.launch``. 

"""

import sys
import select
import rospy
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2023.msg import msg_struct

def publish_msg(msg):
    """
    It takes a message as parameter and extracts the position and the velocity information from it.

    Args:
        msg: message received from the topic /odom. The type is nav_msgs.msg.Odometry
    """

    global publisher

    position = msg.pose.pose.position  # Get the position from the msg
    """Variable to get the position from the msg
    """
    velocity = msg.twist.twist.linear  # Get the twist from the msg
    """Variable to get the twist from the msg
    """

    new_msg = msg_struct()  # Create new message
    """Variable to create a new message 
    """

    new_msg.x = position.x
    """Variable to get the x position from the new message
    """
    new_msg.y = position.y
    """Variable to get the y position from the new message
    """
    new_msg.vel_x = velocity.x
    """Variable to get the x velocity from the new message
    """
    new_msg.vel_y = velocity.y  # here the slides call it vel_z but it might be a typo
    """Variable to get the y velocity from the new message
    """

    publisher.publish(new_msg)  # Publish the message

def feedback_callback(feedback):
    """
    Function to check if the goal is reached by the the robot by comparing the feedback
    status (feedbak.stat) with the string "Target reached!". If the goal is reached, it
    sets the variable finished to True.

    
    Args:
    	feedback
    """
    global finished

    if feedback.stat == "Target reached!":
        finished = True

def start_client():
    """
    Function to connect the action server "/reaching_goal" and waits for it to become
    available. It then prompts the user to enter the goal coordinates (x,y) and sends the goal to the server. If the coordinates are not acceptable, it prints the sentence "please provide both coordinates!". It also sets up a loop to continuosly check for user input to cancel the goal if desired.

    Args:
        None
    """
    global finished

    # connecting to the server /reaching_goal
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():  # get the goal and possibility to delete it
        try:
            x, y = map(float, input("Goal coordinates (x, y): ").split(","))
        except:
            print("please provide both coordinates!")
            continue

        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        finished = False
        client.send_goal(goal, None, None, feedback_callback)

        # The user can cancel the goal by typing 'x' as long as the robot has not reached it yet
        print("\nEnter 'x' to cancel the goal: ", end="")
        while not finished:
            val = select.select([sys.stdin], [], [], 1)[0]

            if val:
                value = sys.stdin.readline().rstrip()

                if value == "x":  # then we must cancel the goal
                    print("Goal has been cancelled!")
                    client.cancel_goal()
                    finished = True  # exit the loop

def main():
    """
    Main function that initializes the ROS node, the publisher is set up for
    "/position_velocity" topic and "/odom" topic gets the current position and velocity information. At the end it is called the previous function "start_client"

    Args:
        None
    """

    global publisher

    # Inizialize the node
    rospy.init_node('action_client')

    publisher = rospy.Publisher("/position_velocity", msg_struct, queue_size=1)
    """Variable to publish the message on the topic /position_velocity
    """

    rospy.Subscriber("/odom", Odometry, publish_msg)

    start_client()

if __name__ == "__main__":
    main()


