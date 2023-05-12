# CapstoneC23
USAFA Robot Teaming Capstone 2022-2023

This repository holds the code to run the SLAM algorithm on the convoy.
The repository for the convoy, specifically the follower bots are located at the repository here: https://github.com/AF-ROBOTICS/Convoy


# User Guide for Class of 24

# SLAMming
to get started slamming:
1. Ensure vlodyne lidar is connected via ethernet port and proper ip configuration is complete IAW ROS velodyne vlp16 tutorial (http://wiki.ros.org/velodyne/Tutorials-Getting%20Started%20with%20the%20Velodyne%20VLP16) 

2. run command in your ros workspace:
	$ roslaunch slam_launch visual_slam.launch
3. This will lauch Rviz where you can see the slam in action
4. profit

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
