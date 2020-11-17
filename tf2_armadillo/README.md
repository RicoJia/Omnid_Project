#tf2_armadillo

**Author: Rico Ruotong Jia (ruotongjia2020@u.northwestern.edu)**

This is an interface tool between ROS ```tf2``` and ```geometry_msgs``` messages and Armadillo matrices. 

So far this is a minimal implementation, as there are more message types that need to be implemented. 

### Conversions and Transforms
1. toMsg from ```arma::Matrix<double>::fixed<4,4>``` to
    - ```geometry_msgs::Transform```
    - ```tf2::Transform```
2. fromMsg to ```arma::Matrix<double>::fixed<4,4>``` from
    - ```geometry_msgs::Transform```
    - ```tf2::Transform```
3. doTransform will to be implemented as needed 

### Usage
Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. 
Depending on your current version of ROS, use:
```
rosdep install --from-paths src --ignore-src --rosdistro noetic
```
