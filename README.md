# RESEARCH TRACK 1 : SECOND ASSIGNMENT.
Ramona Ferrari, S4944096

This is a report for the second assignment of the **Research Track 1** course, for the **Robotics Engineering Master's degree** at the University of Genoa.

## Description of the Simulator
The simulator shows a 3D simulator enviroment where a robot can move. In the enviroment (specified in the file named `sim_w1.launch`) there are some obstacles: the walls.
In the enviroment there are two windows:

- **Rviz**: is a 3-dimensional tool for ROS Visualization. It allows the user to view the
simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. \
By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor
inputs to planned (or unplanned) actions. \
When it starts, three noes are actually executed: `joint_state_publisher` (it allows the package to read the robot_description parameter from the parameter server, finds all of
the non-fixed joints and publishes a JointState message with all those joints defined), `robot_state_publisher` (it uses the URDF specified by the parameter robot_description and the joint
positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results viatf.), `rviz`.

- **Gazebo** is the 3D simulator for ROS. The dynamic simulation is based on various physics engines (ODE, Bullet, Simbody and DART),
  it includes sensors, plugin to customize robots, sensors and the eniroment, realistic rendering of the enviroment and the robots and it allows ROS integration.

### Motion 
The robot may be controlled using ROS topics (/cmd_vel) (a nice tool is teleop_twist_keyboard, which may be
launched with `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`). When moving the robot around,
information coming from sensors may be visualized in Rviz (ex: odom, or cameras). \
The starting point of this assignment can be dowloaded at the following link: `https://github.com/CarmineD8/assignment_2_2023`. Within there are 3 already implemented nodes: \

- `go_to_point_service.py`: it's a service  node which allows the robot to reach a point, moving towards the desired position.
- `bug_as.py`: it is the action server node that receives the desired position from the client and calls the needed services to bring the robot to the desired position.
- `wall_follow_service.py`: it is a service node and it allows the robot to follow the wall if it reaches one along the way.

## Run the Code
The steps to run the code are the following:

- Go inside the `src` folder of ROS workspace, clone the assignment folder: `git clone mettere mio link di repo` and then move to the correct branch, named master, with the command `git checkout master`. 
- From the root directory, digit `roscore` to run the master and then `catkin_make`. This last command builds packages and automates the process of compiling and generating executable files from source code in a catkin workspace.
- If the user has not yet install xterm, it's possible to install it with `sudo apt-get install xterm`. xterm is a terminal emulator for the X Window System, which is a graphical user interface used in Unix-like operating systems. 
- To run the code and see the enviroment: `roslaunch assignment_2_2023 assignment1.launch`.

## Description of the assignment
It's required to create a new package, in which you will develop three nodes:

- (a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
- (b) A service node that, when called, returns the coordinates of the last target sent by the user;
- (c) Another service node that subscribes to the robot’s position and velocity (using the custom message) and
implements a server to retrieve the distance of the robot from the target and the robot’s average speed. \
And then create a launch file to start the whole simulation. Use a parameter to select the size of the averaging window of node (c).

## How the code was developed
The code was developed step by step and the first node implemented was (a). \





 
