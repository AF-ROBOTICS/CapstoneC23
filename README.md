# CapstoneC23
USAFA Robot Teaming Capstone 2022-2023

This repository holds the instructions and code to run the work of the Robot Teaming Capstone of 2022/23. 
The repository for the convoy, specifically the follower bots are located at the repository here: https://github.com/AF-ROBOTICS/Convoy


# User Guide for Class of 24
# Dependencies
following these instructions will help ensure you have all packages running so that you can execute the steps in the sections below.
1. Cole the repo above into your src folder
2. use ros command: rosdep install --from-paths src --ignore-src -r -y
3. profit

# SLAMming
to get started slamming:
1. Ensure vlodyne lidar is connected via ethernet port and proper ip configuration is complete IAW ROS velodyne vlp16 tutorial (http://wiki.ros.org/velodyne/Tutorials-Getting%20Started%20with%20the%20Velodyne%20VLP16) 

2. run command in your ros workspace:
	$ roslaunch slam_launch visual_slam.launch
3. This will lauch Rviz where you can see the slam in action
4. profit

# Navigation
1. clone https://github.com/fazildgr8/ros_autonomous_slam.git into src
2. clone turtlebot3 and turtlebot3 simulation packages into src
3. run: roslaunch slam_launch autonomous_explorer.launch
4. for information on how to run simulation see: https://towardsdatascience.com/ros-autonomous-slam-using-randomly-exploring-random-tree-rrt-37186f6e3568


# Leader Bot Movement
Once the SLAM is running from the steps above, following the next steps will allow the leader bot to start moving in according to the current layot described in the tech doc
1. Ensure wiring is properly connected. The group has had issues in the past with the USB connection from the NUC to the Arduino.
2. Ensure NUC and robot car are both powerd on and untethered, this is inticated by the ESC fan running and and blue light on the NUC. 
3. Open a second tab in the terminal alongside the SLAM
4. Run the command: rosrun leaderbot leaderbot_serial.py
5. Open a third tab
6. Run the command: rosrun leaderbot leader.py
The above code will get the robot moving with a very rough wall following algorithm. To include the OpenMV camera and AprilTag detection:
1. Open a fourth tab
2. Run the command: rosrun  leaderbot apriltag_detect.py
With all of these steps running, the leader will roughly wall follow until one of the cameras sees an AprilTag, then the whole system will stop.

# Microhard
The microhards are set up at the IP addresses taped to them, one as a master and the other as a slave. 
Current setup:
1. 1 antenna on each board (Be careful this connection is delicate)
2. The master is set at the IP address: 192.168.168.11
3. The slave is set up at the IP address: 192.168.168.12
4. These work just as routers after setup

If you wish to restart both boards or start from scratch:
1. Ensure the antenna is connected to the board - be careful as this connection is delicate
2. Locate the button on the side of the board and hold for 10 seconds to reset
2a. This will reset the IP address to 192.168.168.1
3. Follow the user guide labeled here http://www.isnmp.com/download/pmddl2350/pMDDL.Operating%20Manual.v1.2.1.pdf
4. When using this guide execute the quick start manual steps. I was not able to get the quick start (Automatic) to work, so I had to set up the new IP address manually and select slave and master.

# Follower Bot
This robot has the Intel RealSense and the OpenMV camera attached to it.
It uses the OpenMV camera to detect and follow the AprilTag. Setup is according to the diagrams found in the tech doc.
To operate with its current setup:
1. Open a terminal and ssh into the Raspberry Pi 4 on the follower
2. Run the command: roslaunch donkeycar convoy.launch
3. The follower will detect an AprilTag then will steer and maintain speed to ensure proprt following
