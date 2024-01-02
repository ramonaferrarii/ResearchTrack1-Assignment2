# RESEARCH TRACK 1 : SECOND ASSIGNMENT.
Ramona Ferrari, S4944096

This is a report for the second assignment of the **Research Track 1** course, for the **Robotics Engineering Master's degree** at the University of Genoa.

## Description of the Simulator
The simulator shows a 3D simulator enviroment where a robot can move. In the squared arena (specified in the file named `sim_w1.launch`) there are some obstacles: the walls.
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
The starting point of this assignment can be dowloaded at the following link: <a href="https://github.com/CarmineD8/assignment_2_2023">https://github.com/CarmineD8/assignment_2_2023</a>. Within there are 3 already implemented nodes: 

- `go_to_point_service.py`: it's a service node, implemented as a finite state machine, which allows the robot to reach a point, moving towards the desired position and check if the robot manages to reach it.
- `bug_as.py`: it is responsible for determining the actions of the robot based on the "change_state" function. This node imports messages and services from other nodes to facilitate communication amongst various components of the simulation system. These messages include laser scan data, odometry information, twist commands for velocity control, and services for switching navigation modes. Callback functions are utilized to process the data and update the robot's position and orientation. The planning function, which considers obstacles in the environment, serves as a callback for the action server and defines the robot's goal-planning behavior.
- `wall_follow_service.py`: it is a service node and it allows the robot to follow the wall if it reaches one along the way and also to avoid the robot crush into it.

## Run the Code
The steps to run the code are the following:

- Go inside the *src* folder of ROS workspace, clone the assignment folder: `git clone mettere mio link di repo` and then move to the correct branch, named master, with the command `git checkout master`. 
- From the root directory, digit `roscore` to run the master and then `catkin_make`. This last command builds packages and automates the process of compiling and generating executable files from source code in a catkin workspace.
- If the user has not yet install xterm, it's possible to install it with `sudo apt-get install xterm`. xterm is a terminal emulator for the X Window System, which is a graphical user interface used in Unix-like operating systems.
- Please, check if all files in *scripts* folder are executable: go in the *scripts* folder, digit `ls`and if the files are written in green, they are executable; otherwise it's possible to digit `chmod +x "name_of_the_file"` to make it executable. 
- To run the code and see the enviroment: `roslaunch assignment_2_2023 assignment1.launch`. This command runs all the nodes; but if the user wants to run a specific node, it's possible with this command `rosrun assignment_2_2023 "node_name".py`. 

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
The nodes communicates through messages; for this reason in the project there is folder, named *msg* and inside there is the file `msg.struct.msg` which defines the structure of the (custom) ROS message: position ("float 64 x" and "float64 y") and velocity ("float64 vel_x" and "float64 vel_y") of the robot. If a message is included in the project, it's required to modify the `CMakeLists.txt`. \
Then, in the folder *scripts* a file named `action_client.py` is created and this is the pseudo code: 
```python
Import the necessary modules and libraries
Initialization of the ROS node
Initialization of the publisher to publish the message
Initialization of the subscriber to listen the topic /odom and recall of function publish_msg when a new message arrives
Start of function start_client
Initialization of a client and read the coordinates by input
Create an object of type PlanningGoal and set the coordinates of the goal
Set the variable finished = False
Send the goal to server + update of the motion
Until finished is False: every second check if there is input
If there is input, read the value and if the value is x, delete the goal and set finished = Tue (to stop the loop)
```
To accomplish this task, some functions are implemented:

- `publish_msg`: it takes a message as parameter and extracts the position and the velocity information from it. The, it creates a new msg_struct and publishes it.
- `feedback_callback`: this function checks if the goal is reached by the robot by comparing the feedback status (`feedbak.stat`) with the string `Target reached!`. If the goal is reaches, it sets the variable `finished` to True.
- `start_client`: it is the main function; it connects to the action server "/reaching_goal" and waits for it to become available. It then prompts the user to enter the goal coordinates (x,y) and sends the goal to the server. If the coordinates are not acceptable, print the sentence "please provide both coordinates!".  It also sets up a loop to continuously check for user input to cancel the goal if desired.\

In the `main_function` the ROS node is initialized, the publisher is set up for "/position_velocity" topic and "/odom" topic gets the current position and velocity information. At the end it is called the previous function `start_client`. 

In *launch* folder, there are two files: `sim_w1.launch` and `assignment1.launch`. The last one is written in XML format and it is used to simplify the process of starting multiple nodes; for this reason, within this file it's important to delcare all the defined nodes. Here, xterm is used and it allows to open a new terminal where the user can write the goal coordinates and also delete the goal. Please, notice this terminal is used only for this purposes! 

Node (b) is implemented in another file, named `service.py` in *scripts* folder). This node is a service and for this reason a new folder, *srv* is created. Within this folder, the file `last_goal_srv.srv` define the structure of the service file. Usually, a service file is composed by two parts: the *request* and the *responce*. In this type of requested service, the responce is the needed part, because the user wants to see the last coordinates provided. Also the files `CMakeList.txt` and `assignment1.launch` are modified. \
To accomplish the task, some functions are implemented:

- `retrieve_last_goal`: it takes as parameter a message and it retrieves the last goal coordinates from the received goal message and stores them in the global variables "last_goal_x" and "last_goal_y".
- `serve`: this function is a callback function for the service, called *service*. It handles requests for the last goal and it returns the last goal coordinates as a response to the request.

In the `main` function there is the initialization of the ROS node with name `service`, the set up of both service and subscriber and the starting of the node until the node is stopped. \
This service runs with the command `rosservice call /service` on a terminal of Docker.

At the end, node (c) is implemented in a file named `server.py` (in *scripts* folder). Also for this node, `CMakeList.txt` and `assignment1.launch` are modified. \
It is a service, so it requires a service file, named `distance_speed_srv.srv`, (in the *srv* folder), because the user wants to see the distance between robot and goal and the average speed of the robot itself. 
To accomplish the task, some functions are implemented:

- `setPosVel`: it is called whenever a new message is received on the "/position_velocity" topic. It updates the global varibales "current_x" and "current_y" with the new position values from the message. It also computed the magnitude of the velocity vector from the message and appends it to the "averaging_window" list. If the length of the "averaging window" exceeds the specified "window_size" (which is the parameter requested by the assignment), the oldest value is removed from the list.
- `serve`: it is called when a request is made to the service. It retrieves the last goal position using the "service_proxy" and calculates the Euclidean distance between the current position and the last position. The proxy is used to launch the node (b) and using it for the node (c). This function also computes the average speed by summing all the values in the "averaging_window" list and dividing it by the "window_size". It returns the distance and the spees in the response. 

In the `main` function, there is the initialization of the ROS node and it subscribes to the "/position_velocity" topic to receive position and velocity messages. It also initializes the "service_proxy" to access the "/service" service and waits for it to be available. Then it starts a service server for the "/server" service and spins to keep the node running. \
This service runs with the command `rosservice call /server`. For computing the distance and the average speed, the library `math` is imported. \\

To close the enviroment and the whole simulation, press `Ctrl+C` and `gazebo-2` in the same terminal where the command `roslaunch assignment_2_2023 assignment1.launch` was launched. 

## Further Improvements
- The user does not know the dimension of the arena; so it's possible to implement a function that computes this information, to avoid the user to send useless coordinates.
- Improve the direction in which the robots turns when the goal is beyond the wall.
- The velocity of the robot can be changed and also the graphic inteface can be improved.
- Choose a better way to compute the average speed: in this case a fixed dimension list is defined, but it's possible to ask directly the user the dimension of the averaging window.

However, the code accomplish quite well the tasks. 





 
