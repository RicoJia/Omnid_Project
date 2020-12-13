# Omnid Project

Author: Rico Ruotong Jia

### Topics and Actions
#### Subscribed Topics
1. The Omnid simulator provides an interface with Moveit! for motion planning. That works on ROS actions. Note that an external user can publish on
the 'goal' topic to control the robot too. 
    - Action Topics: 
        ```
        /omnid/follow_joint_trajectory/cancel
        /omnid/follow_joint_trajectory/feedback
        /omnid/follow_joint_trajectory/goal
        /omnid/follow_joint_trajectory/result
        /omnid/follow_joint_trajectory/status
        ``` 
2. Also, there is a test mode: set ```test_mode_on``` to true in ```params.yaml```, then you can control
each joint by publishing on ```/joint_states_control```

#### Published Topics
1. ```/joint_states``` lists all joint states for motion planning. 
### Modes of Operation
#### Joint Control Test 
Direct joint level control, see [Subscribed Topics](#Subscribed-Topics) for how to enable it.
#### After Spring Joint Test
Direct control on the after spring joint (theta). 

### Additional Packages
These packages are optional to install, and they are not essential to the operation of 
the delta arm. However, based on your need, you might consider using them: 
1. ros-noetic-joint-state-publisher-gui
