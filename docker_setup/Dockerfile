# This dockerfile sets up the build environment for the omnid project
# Read through it to see what is needed to setup the environment

# Use a docker image provided by ros
FROM ros:noetic-ros-core-focal

# ubuntu packages that are needed
# Note: some of these packages are already covered when
# doing a full desktop install of ubuntu/ros on the omnid computer
RUN dpkg --add-architecture i386 \
    && apt-get update -yq \
    && apt-get install -yq --no-install-recommends \
    build-essential \
    curl \
    dbus \
    doxygen \
    doxygen-latex \
    git  \
    gcc-arm-none-eabi \
    gdb-multiarch \
    libarmadillo-dev \
	liblapack-dev \
    libc6:i386 \
    libopenblas-dev \
    libnewlib-arm-none-eabi \
    libv4l-dev \
    libstdc++-arm-none-eabi-newlib \
    minicom \
    openocd \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
	ros-noetic-camera-info-manager \
    ros-noetic-catch-ros \
    ros-noetic-diagnostic-updater \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rqt-console \
    ros-noetic-rviz \
    ros-noetic-self-test \
    ros-noetic-usb-cam \
    ros-noetic-xacro \
	ros-noetic-moveit-simple-controller-manager \
	ros-noetic-moveit \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

#install joint_state_publisher_gui
RUN apt update \
    && apt upgrade -y \
    && apt install ros-noetic-joint-state-publisher-gui


RUN pip3 install git+https://github.com/catkin/catkin_tools.git \
	&& pip3 install pybullet


# install the ti-cgt-arm compiler (Optional as we have mainly been using gcc now)
RUN curl -L http://software-dl.ti.com/codegen/esd/cgt_public_sw/TMS470/20.2.1.LTS/ti_cgt_tms470_20.2.1.LTS_linux_installer_x86.bin > /ti-cgt.run \
    && chmod 755 /ti-cgt.run \
    && /ti-cgt.run --unattendedmodeui none --mode unattended --prefix /opt/ti \
    && rm /ti-cgt.run
    
    
