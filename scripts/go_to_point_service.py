#! /usr/bin/env python

"""
.. module: go_to_point_service
:platform unix
:synopsis: Python module for motion planning of the robot.
.. moduleauthor:: Ramona Ferrari

ROS node for controlling the robot

Subscribes to:
	**/odom** (nav_msgs/Odometry): 
        the current position/orientation/motion of the robot. 
        
Publishes to:
	**/cmd_vel** (geometry_msgs/Twist): 
        command given to the robot
    	**/current_cmd_vel** (geometry_msgs/Twist): 
        a copy of the command is sent to jupyter.

"""

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time

import math

active_ = False

# robot state variables
position_ = Point()
"""geomtry_msgs/Point: current robot position """
yaw_ = 0
"""Float: current orientation of the robot about the vertical axis (z)"""
# machine state
state_ = 0
"""Int: current state of the state machine"""
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
"""Float: first precision for the orientation.

In order to reach the final orientation, the node checks if
the difference between the desider orientation and the target
orientation is under a certain tolerance, i.e. this number. 

+/- 20 degree allowed."""
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
"""Float: second precision for the orientation

In order to reach the final orientation, the node checks if
the difference between the desider orientation and the target
orientation is under a certain tolerance, i.e. this number. 

+/- 2 degree allowed. More precision compared to the other
precision value.
"""
dist_precision_ = 0.3
"""Float: precision for the distance from a target.

While trying to go at a certain position, the system checks 
regularly if the distance between the current (x,y) position
and the target one is below a certain threshold, that is this 
number. 
"""

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
"""Float: Gain yaw error to z angular velocity

In ROS Noetic, it may be necessary to change the sign of this proportional controller 
"""
kp_d = 0.2
ub_a = 0.6
"""Float: maximum robot angular velocity. """
lb_a = -0.5
"""Float: minimum angular velocity of the robot. """
ub_d = 0.6
"""Float: maximum linear velocity of the robot. """

# publishers
pub = None
"""ROS_publisher_handle: publisher to /cmd_vel """

# service callbacks


def go_to_point_switch(req):
    """
    Callback for the go_to_point_switch service.
    
    Args:
    	req (SetBoolRequest): Request object for the service.

    Returns:
        SetBoolResponse: Response object for the service. 
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks


def clbk_odom(msg):

"""
    Callback for the odometry subscriber.

    Args:
        msg (nav_msgs/Odometry): Odometry message received.
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
"""
    Change the state of the machine.

    Args:
        state (int): New state to change to.
    """
    
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    Normalize the given angle to be within -pi to pi range.

    Args:
        angle (float): Angle to normalize.

    Returns:
        float: Normalized angle.
    """
    
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """
    Adjust the robot's yaw to point towards the desired position.

    Args:
        des_pos (Point): Desired position to orient towards.
    """
    
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    Move the robot straight towards the desired position.

    Args:
        des_pos (Point): Desired position to move towards.
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d*(err_pos)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    """
    Stop the robot when the task is done.
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
                

def main():
    """
    Main function to run the ROS node for controlling the robot.
    There is also the instanciation of the channels.
    :global pub: Publisher object for /cmd_vel topic.
    :global active_: Boolean flag indicating whether the robot is active or not.
    
    Parameters:
    	None
    	
    Returns:
    	None
    	
    """
    
    global pub, active_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            desired_position_.x = rospy.get_param('des_pos_x')
            desired_position_.y = rospy.get_param('des_pos_y')
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
