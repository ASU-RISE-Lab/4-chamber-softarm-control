# vrpn_client_ros
This fork aims at porting the original code from the kinetic-devel branch to ROS2 Foxy.

## Requirements

The code requires VRPN to work. Please git clone the following and build in the colcon_ws
```
    git clone https://github.com/ASU-RISE-Lab/vrpn.git
```
After building the above vrpn package, source your workspace. Next proceed to install the vrpn_client_ros

## What works?
The original package is from here https://github.com/zeroos/vrpn_client_ros/tree/foxy-devel by Author: @zeroos. The current package has a modified CMAkeLists to include necessary inclusions for the ROS2 Foxy edition. 
