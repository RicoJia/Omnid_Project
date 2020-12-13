# Omnid Simulator and Moveit! Packages


This is the main repository of an Omnid group simulator and its Moveit! motion planning pipeline. 
If you haven't done so, check out my [**blog post**](https://ricoruotongjia.medium.com/s-d65d8ffcc73d) for some higher level implementation detals. 

![Hnet-image](https://user-images.githubusercontent.com/39393023/101986502-bfa2c600-3c53-11eb-8a7a-8cc360877151.gif)

There are 3 Delta robots and one object platform in this project. 
![1*c-ZS4OG450Jem4atBD1XIQ](https://user-images.githubusercontent.com/39393023/101987475-eebc3600-3c59-11eb-9791-72eb11737a46.png)

The simulator includes: 
 - Robot model, including URDF and PyBullet model 
 - Joint control and joint information output   
 - Torsion springs mounted on each robot as serial elastic actuators
 - Validation of torsion springs.  

![Screenshot from 2020-12-11 18-48-34](https://user-images.githubusercontent.com/39393023/101987454-cc2a1d00-3c59-11eb-96fb-80ca5ef36e69.png)

The motion planning pipeline includes
 - `omnid_moveit_config`: a properly configured Moveit! configuration package
 - Robot URDF model: URDF specifically for Moveit! 
 - Plugins:
   - omnid_move_group_interface: updates robot poses when the object's interactive marker is clicked and dragged
   - omnid_kinematics_plugin: IK plugin for a single delta robot
   - omnid_planning_adapter_plugin: post-processing planner adapter that generates the right trajectory waypoints
 - Supplementary Packages:
   - control_msgs: this is the ROS [control_msgs package](https://wiki.ros.org/control_msgs), but as of Dec.2020 it is not 
   available in Ubuntu 20.04 apt store yet. 
   - tf2_armadillo: transformations for common Armadillo matrices and transform datatypes in ROS tf2 and geometry_msgs   
 ![](https://media.giphy.com/media/6MiY7FDVTuBx0ZuSn1/giphy.gif)
 
 There's also some files for setting up the docker in `docker_setup`
  - docker file: docker image file
  - dockint: tool to build a docker container. Created by Dr. Matt Elwin
  - ros_settings.bash: settings to source when starting the docker file
  
### Build the Package
The repo runs on Ubuntu Linux 20.04, ROS Noetic, rosdep, catkin_tools and several other prerequisites. 
As a universal way to setup the workspace, 

1. Create workspace
      ``` shell script
        mkdir -p omnid/src
        cd omnid/src
        git clone https://github.com/RicoJia/Omnid_Project.git
   ```
2. Pull Dependencies 
   - ```cd Omnid_Project``` Go to the project directory
   - Install VCS tool, [see here](https://github.com/dirk-thomas/vcstool)
   - Use VCS tool to download all necessary packages. **If VCS tool does not work, try git clone all pacakges in ```docker_setup/omnid_docs.repos``` manually.**   
        ```shell script
           vcs import < docker_setup/omnid_docs.repos
        ```
   - ```cd ../..``` go to the root of the package

3. Build a docker container and start it(Please use this dockerfile as it contains the latest dependencies we need)
   - Build the image
      ```
            cp src/Omnid_Project/docker_setup/dockint .
            cp src/Omnid_Project/docker_setup/Dockerfile .
            ./dockint from omnid $(pwd)/src/Omnid_Project/docker_setup
       ``` 

   - Build a container. Thanks [Dr.Matt Elwin](https://robotics.northwestern.edu/people/profiles/faculty/elwin-matt.html) for sharing his dockint tool!
      ``` 
        ./dockint start omnid $(pwd)
        ./dockint run omnid bash
      ```   
    
4. Build the package in the docker container  
   - ```source /opt/ros/noetic/setup.bash``` this will setup some initial settings 
   - ```catkin build``` build this package
   - ```source devel/setup.bash``` Source the workspace
   - ```source src/Omnid_Project/docker_setup/ros_settings.bash```   Source some correct ros settings that overwrite the previous ones
   - ```roslaunch omnid omnid.launch ``` Launch the project
   - Have fun planning!

5. Once done with the project
   - ```exit``` to exit the container
   - ```./dockint stop omnid``` to stop the container

### Basic Usage
On Rviz, 
1. Click and drag the marker embedded in the object platform to a desired pose
    - If we first start at unachievable poses, we can drag the marker anywhere we want until we reach the first achievable poses.   
    - If we try to reach a valid pose to an unachievable pose, the marker will jump back to the closest valid pose
2. Hit plan to see the plan
3. Hit plan and execute to execute the plan
    - The simulator will return fail only if the action request from Moveit! is preempted.  
    This is because Moveit! checks the proximity between the plan and the actual robot TF. Also it checks if the controller takes
    too long to execute a path. see `move_group/trajectory_execution/execution_duration_monitoring`
    
### Video Demonstration
![(https://user-images.githubusercontent.com/39393023/101986502-bfa2c600-3c53-11eb-8a7a-8cc360877151.gif)](https://www.youtube.com/embed/mwzxtJbuQ-Y)