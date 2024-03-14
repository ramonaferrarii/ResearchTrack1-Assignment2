#!/usr/bin/env python3

"""
.. module:: ResearchTrack_Assignment2_server

:platform: Unix
:synopsis: Python service2 for ResearchTrack_Assignment2

.. moduleauthor:: Ramona Ferrari S4944096@studenti.unige.it

This node implements a service to subscribe to the robot's position and velocity.
It uses the custom message. 
It also implements a server to retrieve the distance of the robot from the target and the 
robot's average speed.

There are severals functions implemented:
	*setPosVel*;
	*serve*.
	
The command to run the service is ``rosservice call /server``.
"""

import math
import rospy
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from std_srvs.srv import *
from assignment_2_2023.msg import msg_struct
from assignment_2_2023.srv import distance_speed_srv, last_goal_srv, distance_speed_srvResponse

# Global variables 
current_x = 0
"""Global variable to store the current position of robot on x coordinate
"""
current_y = 0
"""Global variable to store the current position of robot on y coordinate
"""
averaging_window = []
"""List to store the average speed of the robot in the last "window_size" messages received
"""
window_size = 20
"""Global variable used as parameter to select the size of the averaging window of the node.
"""
service_proxy = None
"""Global variable to store the service proxy to access the "/service" service.
"""

# Function to set current position and current velocity of the robot
def setPosVel(msg):
    """
    Function called whenever a new message is received on the "/position_velocity" topic.
    It updates the global variables "current_x" and "current_y" with the new position values
    from th message and appends it to the "averaginf_window" list.
    If the lenght of the "averagin_window" exceeds the specified "window_size", the oldest
    value is removed from the list.
    
    Args:
    msg: it contains the average speed and the distance between target and robot.
    """
    global current_x, current_y, averaging_window, window_size

    current_x = msg.x
    current_y = msg.y
    if len(averaging_window) >= window_size:
        averaging_window.pop(0) # then we remove the oldest value
    
    averaging_window.append(math.sqrt(msg.vel_x**2 + msg.vel_y**2))
    
# Handle to service
def serve(req):
    """
    Function called when a request is made to the service. It retrieves the last goal
    position. 
    This function also computes the average speed by summing all the values in the
    "averaging_window" list and dividing it by the "window_size". It returns the distance and
    the speed in the response.
    
    Args:
    req: actually not reqeusted.
    
    
    """
    global current_x, current_y, averaging_window, window_size, service_proxy

    response = service_proxy()
    current_dist = math.sqrt((current_x - response.x)**2 + (current_y - response.y)**2)

    return distance_speed_srvResponse(current_dist, sum(averaging_window) / window_size)


def main():
    """
    Main function to initialize the ROS node, subscribes to "/position_velocity" topic. 
    Initialization of window_size
    It initializes the "service_proxy" to access "/service" service.
    Then it starts a service server for the "/server" and spins to keep the node running.
    
    """

    global window_size, service_proxy

# Get value of averagin_window
    window_size = int(rospy.get_param('/window_size', '20'))
    """ Get the value of the parameter "window_size" from the ROS parameter server.
    """
    # Inizialize the node
    rospy.init_node('server')
    
    rospy.Subscriber("/position_velocity", msg_struct, setPosVel)

# Proxy which allows to get the last position, launching /service
    service_proxy = rospy.ServiceProxy('/service', last_goal_srv)
    """Service proxy to access the "/service" service.
    """
    rospy.wait_for_service('/service')

    rospy.Service("/server", distance_speed_srv, serve)

    rospy.spin()
    

if __name__ == "__main__":
    main()
