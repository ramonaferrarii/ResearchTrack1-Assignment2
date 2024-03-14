.. ResearchTrack_Assignment2 documentation master file, created by
   sphinx-quickstart on Mon Mar 11 09:40:24 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

=====================================================
Welcome to ResearchTrack_Assignment2's documentation!
=====================================================
This is a report for the second assignment of the Research Track 2 course, for the Robotics Engineering Master's degree at the University of Genoa.   
It's required to create a new package, in which you will develop your code. The code was developed for Research Track 1 course and now it is properly commented using *Sphinx*, a tool for creating documentation.

Indices and tables:
===================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Sphinx:
~~~~~~~
Sphinx is an open source documentation tool that allows you to create detailed technical documentation in various formats, such as HTML, PDF, and EPUB. It is widely used to document software projects, APIs, and libraries by writing documents in reStructuredText markup format.

Sphinx offers a range of advanced features and it is highly flexible and customizable through the use of themes and extensions, which allow you to customize the appearance and functionality of the documentation.

Sphinx is a very useful tool for developers, technical writers, and documentation teams who want to create well-structured and easily navigable documentation for their projects.

To install:

- ``sudo apt-get install python3-sphinx``

- ``pip3 install breathe``

- ``pip3 install sphinx-rtd-theme`` 

In the directory where you have the code to comment, run:

- ``sphinx-quickstart``

.. toctree::
   :maxdepth: 2
   :caption: Contents:

Description of the Simulator:
=============================
The simulator shows a 3D simulator enviroment where a robot can move. In the squared arena (specified in the file named sim_w1.launch) there are some obstacles: the walls. In the enviroment there are two windows:

* Rviz: is a 3-dimensional tool for ROS Visualization. It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information. By visualizing what the robot is seeing, thinking, and doing, the user can debug a robot application from sensor inputs to planned (or unplanned) actions. When it starts, three noes are actually executed: *joint_state_publisher* (it allows the package to read the robot_description parameter from the parameter server, finds all of the non-fixed joints and publishes a JointState message with all those joints defined), *robot_state_publisher* (it uses the URDF specified by the parameter robot_description and the joint positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results viatf.), *rviz*.

* Gazebo is the 3D simulator for ROS. The dynamic simulation is based on various physics engines (ODE, Bullet, Simbody and DART), it includes sensors, plugin to customize robots, sensors and the eniroment, realistic rendering of the enviroment and the robots and it allows ROS integration.


Motion:
=======
The robot may be controlled using ROS topics (*/cmd_vel*) (a nice tool is teleop_twist_keyboard, which may be launched with rosrun teleop_twist_keyboard teleop_twist_keyboard.py). When moving the robot around, information coming from sensors may be visualized in Rviz (ex: odom, or cameras).
The starting point of this assignment can be dowloaded at the following link: `<https://github.com/CarmineD8/assignment_2_2023>`. 

Within there are 3 already implemented nodes:

* go_to_point_service.py: it's a service node, implemented as a finite state machine, which allows the robot to reach a point, moving towards the desired position and check if the robot manages to reach it.
* bug_as.py: it is responsible for determining the actions of the robot based on the *change_state* function. This node imports messages and services from other nodes to facilitate communication amongst various components of the simulation system. These messages include laser scan data, odometry information, twist commands for velocity control, and services for switching navigation modes. Callback functions are utilized to process the data and update the robot's position and orientation. The planning function, which considers obstacles in the environment, serves as a callback for the action server and defines the robot's goal-planning behavior.
* wall_follow_service.py: it is a service node and it allows the robot to follow the wall if it reaches one along the way and also to avoid the robot crush into it.

Run the Code:
=============
The steps to run the code are the following:

- ``git clone my_repo's_link``

- ``git checkout master``

- ``roscore``  

- ``catkin_make`` 

- ``sudo apt-get install xterm``

- ``roslaunch assignment_2_2023 assignment1.launch.`` 

 Please, check if all files in scripts folder are executable and make them executable: 
 
 - ``lsand`` 
 
 - ``chmod +x name_of_the_file``
 
Description of the Assignment:
==============================
It's required to create a new package, in which you will develop three nodes:

(a) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the feedback/status of the action server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom;
(b) A service node that, when called, returns the coordinates of the last target sent by the user;
(c) Another service node that subscribes to the robot's position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot's average speed. And then create a launch file to start the whole simulation. Use a parameter to select the size of the averaging window of node (c).


Node (a): Action Client to set target or cancel it.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. automodule:: scripts.action_client
   :members: 

Node (b) : Service node to return last target.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. automodule :: scripts.service
   :members:

Node (c) : Service node to subscribe robot's position and velocity and retrieve distance of the robot.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. automodule :: scripts.server
   :members:
   

