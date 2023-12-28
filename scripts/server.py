#!/usr/bin/env python3
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
current_y = 0
averaging_window = []
window_size = 20
service_proxy = None

# Function to set curretn position and current velocity of the robot
def setPosVel(msg):
    global current_x, current_y, averaging_window, window_size

    current_x = msg.x
    current_y = msg.y
    if len(averaging_window) >= window_size:
        averaging_window.pop(0) # then we remove the oldest value
    
    averaging_window.append(math.sqrt(msg.vel_x**2 + msg.vel_y**2))
    
# Handle to service
def serve(req):
    global current_x, current_y, averaging_window, window_size, service_proxy

    response = service_proxy()
    current_dist = math.sqrt((current_x - response.x)**2 + (current_y - response.y)**2)

    return distance_speed_srvResponse(current_dist, sum(averaging_window) / window_size)


def main():

    global window_size, service_proxy

# Get value of averagin_window
    window_size = int(rospy.get_param('/window_size', '20'))
    # Inizialize the node
    rospy.init_node('server')
    
    rospy.Subscriber("/position_velocity", msg_struct, setPosVel)

# Proxy which allows to get the last position, launching /service
    service_proxy = rospy.ServiceProxy('/service', last_goal_srv)
    rospy.wait_for_service('/service')

    rospy.Service("/server", distance_speed_srv, serve)

    rospy.spin()
    

if __name__ == "__main__":
    main()
