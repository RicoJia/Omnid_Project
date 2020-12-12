# Omnid Simulator and Moveit Packages
![](https://miro.medium.com/max/600/1*aVR3_S5KSilwzJTvuMUfZA.gif)


### Usage
The repo runs on Ubuntu Linux 20.04, ROS Noetic, rosdep, catkin_tools and several other prerequisites. 
As a universal way to setup the workspace, 

1. Create workspace
      ``` shell script
        mkdir -p omnid/src
        cd omnid/src
        git clone https://github.com/RicoJia/Omnid_Project.git
   ```
2. Pull Dependencies
    ```shell script
         cd Omnid_Project
    ```
   - Install VCS tool, [see here](https://github.com/dirk-thomas/vcstool)
   - Use VCS tool to download all necessary packages
        ```shell script
           vcs import < /docker_setup/omnid_docs.repos
        ```
   - if the above does not work, try git clone all pacakges manually.   
   - ```cd ../..``` go to root of the package

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
    
4. Build the package in the docker  
   - ```source src/Omnid_Project/docker_setup/ros_settings.bash``` this will setup some initial settings 
   - ```catkin build``` build this package
   - ```source devel/setup.bash```   Source the workspace
   - ```roslaunch omnid omnid.launch ``` Launch the project
   - Have fun planning!


