# CapstoneC23
USAFA Robot Teaming Capstone 2022-2023

This repository holds the code to run the SLAM algorithm on the convoy.
The repository for the convoy, specifically the follower bots are located at the repository here: https://github.com/AF-ROBOTICS/Convoy

# slamming
to get started slamming:
1. Ensure vlodyne lidar is connected via ethernet port and proper ip configuration is complete IAW ROS velodyne vlp16 tutorial (http://wiki.ros.org/velodyne/Tutorials-Getting%20Started%20with%20the%20Velodyne%20VLP16) 

2. run command in your ros workspace:
	$ roslaunch slam_launch visual_slam.launch
	
3. profit
