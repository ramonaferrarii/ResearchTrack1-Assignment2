#!/usr/bin/env python3

"""
.. module:: ResearchTrack_Assignment2_service

:platform: Unix
:synopsis: Python service1 for ResearchTrack_Assignment2

.. moduleauthor:: Ramona Ferrari S4944096@studenti.unige.it


This node implements a service that, when explicitly called, returns the coordinates of the last target sent by the user.

There are severals functions implemented:
    *retrieve_last_goal*;
    *serve*.
       
The command to run the service is ``rosservice call /service``. 
"""
import rospy
from assignment_2_2023.srv import last_goal_srv, last_goal_srvResponse
import actionlib
import actionlib.msg
import assignment_2_2023.msg

# Define global variables to store last goal coordinates
last_goal_x = 0
"""Global variable to store the last goal on x coordinate
"""
last_goal_y = 0
"""Global variable to store the last goal on y coordinate
"""

# Retrieve the last goal coordinates from the goal message 
def retrieve_last_goal(msg):
    """
    Function that retrieves the last goal coordinates from the received goal message and 
    stores them in the global variables "last_goal_x" and "last_goal_y". 
    
    Args:
    msg: custom message containing the actual position and velocity of the robot. 
    The type of the message is assignment_2_2023.msg.
    """
    global last_goal_x, last_goal_y
    last_goal_x = msg.goal.target_pose.pose.position.x
    last_goal_y = msg.goal.target_pose.pose.position.y

# Service callback function to handle requestes for the last goal
def serve(req): # note: req is not used
    """
    Callback function to handle requestes for the last goal.
    
    Args: 
    req: actually not requested.
    """
    global last_goal_x, last_goal_y
    return last_goal_srvResponse(last_goal_x, last_goal_y)

def main():
    """
    Main function to initialize the node and set up of the service and subscriber.
    
    Args:
    None
    """
    # Initialize the node 
    rospy.init_node("service")
    # Handle request for the last goal
    rospy.Service("service", last_goal_srv, serve)
    # Subscriber to retrieve the last goal coordinates
    rospy.Subscriber("/reaching_goal/goal", assignment_2_2023.msg.PlanningActionGoal, retrieve_last_goal)

    rospy.spin()

if __name__ == "__main__":
    main()

