# Omnid Simulator and Moveit Packages

### Usage
1. Create workspace
      ``` shell script
        mkdir -p omnid/src
        cd omnid/src
        git clone https://github.com/RicoJia/Omnid_Project.git
        cd .. 
   ```
2. Build a docker container and start it(Please use this dockerfile as it contains the latest dependencies we need)
   - Build the image
      ```
            cp src/Omnid_Project/docker_setup/dockint .
            cp src/Omnid_Project/docker_setup/Dockerfile .
            ./dockint from omnid $(pwd)/src/Omnid_Project/docker_setup
       ``` 

   - Build a container
      ``` 
        ./dockint start omnid $(pwd)
        ./dockint run omnid bash
      ```   
3. Pull Dependencies
    
4. Build the package in the docker  
   - ```source ros_settings.bash``` this will setup some initial settings 
   - ```catkin build``` build this package
   - ```source devel/setup.bash```   Source the workspace
   - ```roslaunch omnid omnid.launch ``` Launch the project
   - Have fun planning!


