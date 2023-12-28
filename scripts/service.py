#!/usr/bin/env python3
import rospy
from assignment_2_2023.srv import last_goal_srv, last_goal_srvResponse
import actionlib
import actionlib.msg
import assignment_2_2023.msg

# Define global variables to store last goal coordinates
last_goal_x = 0
last_goal_y = 0

# Retrieve the last goal coordinates from the goal message 
def retrieve_last_goal(msg):
    global last_goal_x, last_goal_y
    last_goal_x = msg.goal.target_pose.pose.position.x
    last_goal_y = msg.goal.target_pose.pose.position.y

# Service callback function to handle requestes for the last goal
def serve(req): # note: req is not used
    global last_goal_x, last_goal_y
    return last_goal_srvResponse(last_goal_x, last_goal_y)

def main():
# Initialize the node 
    rospy.init_node("service")
# Handle request for the last goal
    rospy.Service("service", last_goal_srv, serve)
# Subscriber to retrieve the last goal coordinates
    rospy.Subscriber("/reaching_goal/goal", assignment_2_2023.msg.PlanningActionGoal, retrieve_last_goal)

    rospy.spin()


if __name__ == "__main__":
    main()
