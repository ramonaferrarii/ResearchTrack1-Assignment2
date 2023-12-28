#!/usr/bin/env python3
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

    global publisher

    position = msg.pose.pose.position # Get the position from the msg
    
    velocity = msg.twist.twist.linear # Get the twist from the msg

    new_msg = msg_struct() # Create new message
    
    new_msg.x=position.x
    new_msg.y=position.y
    new_msg.vel_x=velocity.x
    new_msg.vel_y=velocity.y # here the slides call it vel_z but it might be a typo
    
    publisher.publish(new_msg) # Publish the message


def feedback_callback(feedback):
    global finished

    if feedback.stat == "Target reached!":
        finished = True
	
def start_client():
    
    global finished
    # connecting to the server /reaching_goal
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown(): # get the goal and possibility to delete it
        
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
                        
                if (value == "x"):  # then we must cancel the goal
                    print("Goal has been cancelled!")
                    client.cancel_goal()
                    finished = True # exit the loop
            



def main():

    global publisher
    # Inizialize the node
    rospy.init_node('action_client')
    
    publisher = rospy.Publisher("/position_velocity", msg_struct, queue_size=1)

    rospy.Subscriber("/odom", Odometry, publish_msg)

    start_client()
    

if __name__ == "__main__":
    main()
