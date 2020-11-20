# Omnid Simulator and Moveit Packages

### Usage
Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. 
Depending on your current version of ROS, use:
```
rosdep install --from-paths src --ignore-src -r -y
```

This is particularly useful for downloading moveit_visual_tools since as of November 10, 2020, it's not
available on apt.  
