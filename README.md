# US&R Simplified 

This project simulates a simplified Urban Search and Rescue (US&R) system using ROS and two Turtlebot robots.

## System Overview

- Explorer robot maps environment and finds victim locations (ArUco markers)
- Follower robot rescues victims by visiting markers in ID order
- Custom broadcaster and listener nodes handle transform frames
- Sorting algorithm orders marker locations before rescue

## Implementation 

- Launched robots in Gazebo environment with 4 ArUco markers
- Implemented broadcaster node to publish marker frame transforms
- Listener node converts marker poses to map frame and stores locations
- Bubble sort used to order stored locations by marker ID
- Follower visits markers sequentially using stored poses

## Results

- Explorer successfully mapped environment and detected all markers
- Follower able to autonomously navigate to all 4 markers in correct order
- System completes full victim discovery and rescue simulation

## Challenges

- Handle timing between publishing and subscribing nodes 
- Debugging frame transform issues
- Implementing modular OOP structure

## Conclusion

- Developed a working multi-robot US&R system with ROS
- Gained experience with ROS transform frames, callbacks, parameters
- Foundation for expanding to more complex USAR scenarios

[Project Report](report.pdf)
