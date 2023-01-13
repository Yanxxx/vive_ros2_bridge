# HTC Vive controller ROS2 bridge for Ubuntu22.04
## Description 
Broadcast HTC devices locatioin and quaternion with /tf of ROS2. 

Please feel free to use it. 


### Install and usage:

This is a ROS2 package. Just place it under the 
```bash
ros2_ws/src/
```
of ROS2 working directory. 
Then colcon build.


### Use without Helmet

Locate steamvr setting file, it is usually under steam install location. 
```bash
~/.local/share/Steam/steamapps/common/SteamVR/resources/settings
```
open default.vrsettings
change the item 
```bash
{"requireHmd": true,} 
```
to 
```bash
{"requireHmd": false,}
```
Restart the steamvr.

