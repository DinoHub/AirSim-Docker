### Instructions for Docker
## build image from this dockerfile. E.g. IMAGE_NAME = ue4, IMAGE_TAG = Tartan
# docker build -t <IMAGE_NAME>:<IMAGE_TAG> --build-arg GITHUB_USER=<user> --build-arg GITHUB_PWD=<password> .

## run a container from the built image. vol should include UE4_env and TartanAir Codes
# xhost +local:docker && docker run --rm -it -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/path/to/your/vol:/workspace/TartanAir" -e "DISPLAY=${DISPLAY}" --ipc="host" <IMAGE_NAME>:<IMAGE_TAG>
# $ ./UnrealEngine/Engine/Binaries/Linux/UE4Editor /workspace/UnrealProj/TartanTest.uproject
# "Would you like to rebuild AirSim?" >> Yes

## save the container settings
# docker container ls -a		(to get CONTAINER_NAME)
# docker commit <CONTAINER_NAME> <IMAGE_NAME>:<IMAGE_TAG> after first run of ue to prevent long ue init each time from recompiling shader maps

## Open another bash
# docker exec -it <CONTAINER_NAME> bash

## Open another bash with display
# xhost +local:docker && docker exec -it -e "DISPLAY=${DISPLAY}" <CONTAINER_NAME> bash


FROM adamrehn/ue4-build-prerequisites:cudagl10.0
SHELL ["/bin/bash", "-c"]

USER root


# ==============================
# Configurable params
# ==============================
ARG GITHUB_USER
ARG GITHUB_PWD
RUN env


# ==============================
# Replace with local SG mirrors
# ==============================
RUN sed --in-place --regexp-extended "s/(\/\/)(archive\.ubuntu)/\1sg.\2/" /etc/apt/sources.list
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ Asia/Singapore
RUN apt-get update
RUN apt-get install -y --no-install-recommends sudo curl tzdata
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && sudo dpkg-reconfigure -f noninteractive tzdata


# ==============================
# UI Support
# ==============================
# Enable Vulkan support
RUN sudo apt-get install -y --no-install-recommends libvulkan1 && \
	VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
	mkdir -p /etc/vulkan/icd.d/ && \
	echo \
	"{\
		\"file_format_version\" : \"1.0.0\",\
		\"ICD\": {\
			\"library_path\": \"libGLX_nvidia.so.0\",\
			\"api_version\" : \"${VULKAN_API_VERSION}\"\
		}\
	}" > /etc/vulkan/icd.d/nvidia_icd.json

# Enable X11 support (including the libraries required by CEF) and xvfb so we can create a dummy display if needed
RUN sudo apt-get install -y --no-install-recommends \
	libasound2 \
	libatk1.0-0 \
	libcairo2 \
	libfontconfig1 \
	libfreetype6 \
	libglu1 \
	libnss3 \
	libnspr4 \
	libpango-1.0-0 \
	libpangocairo-1.0-0 \
	libsm6 \
	libxcomposite1 \
	libxcursor1 \
	libxi6 \
	libxrandr2 \
	libxrender1 \
	libxss1 \
	libxv1 \
	x11-xkb-utils \
	xauth \
	xfonts-base \
	xkb-data \
	xvfb


# ============================== 
# User Setup
# ==============================
# Add a user with the same user_id as the user outside the container
# =======
ARG USERID=1000
ARG GROUPID=1000
# Use 'ue4' for username and group name
ENV USERNAME=ue4
RUN echo "$USERNAME:$USERNAME" | chpasswd \
&& adduser $USERNAME sudo \
&& echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME
WORKDIR /home/$USERNAME
RUN sudo chown --recursive $USERNAME:$USERNAME /home/$USERNAME
USER $USERNAME


# ============================== 
# Installs
# ==============================

# ==============================
# Unreal Engine 4.25
# ==============================
ARG FOLDER_NAME=workspace
WORKDIR /$FOLDER_NAME
RUN sudo chown -R $USERNAME:$USERNAME /$FOLDER_NAME
RUN git clone -b 4.25 https://$GITHUB_USER:$GITHUB_PWD@github.com/EpicGames/UnrealEngine.git
# =======
WORKDIR /$FOLDER_NAME/UnrealEngine
RUN ./Setup.sh
RUN ./GenerateProjectFiles.sh
USER $USERNAME
RUN make


# ==========================
# ROS Melodic
# ==========================
WORKDIR /home/$USERNAME 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
# Note: Option 1 tested to work but there are cases where it fails, prompting for pre-requisites to be installed, use Option 2 in this case.
# Option 1: Install ros-melodic-desktop-full 
RUN sudo apt-get update && sudo apt-get install ros-melodic-desktop-full -y --no-install-recommends

# Option 2: Install pre-requisites for ros-melodic-desktop-full before install ros-melodic-full
# RUN sudo apt-get update && sudo apt-get install ros-melodic-perception ros-melodic-simulators ros-melodic-urdf-sim-tutorial ros-melodic-perception-pcl ros-melodic-pcl-conversions ros-melodic-pcl-ros libgazebo9-dev ros-melodic-gazebo-dev ros-melodic-gazebo-plugins ros-melodic-gazebo-ros ros-melodic-gazebo-ros-pkgs -y --no-install-recommends
# RUN sudo apt-get update && sudo apt-get install ros-melodic-desktop-full -y --no-install-recommends

# Additional Dependencies
RUN sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server ros-melodic-dynamic-edt-3d -y --no-install-recommends
RUN sudo apt-get install ros-melodic-catkin ros-melodic-teleop-twist-keyboard python-pip python-setuptools python-wstool python-catkin-tools -y --no-install-recommends
RUN sudo apt-get install ros-melodic-cmake-modules 
# Add to bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Cleanup
RUN sudo apt-get clean autoremove


# =======================================================================
#  AirSim v1.4.0-linux (https://microsoft.github.io/AirSim/build_linux/)
# =======================================================================
WORKDIR /$FOLDER_NAME
# v1.4 is working. Stick to v1.4 first.
RUN git clone --depth 1 -b v1.4.0-linux https://github.com/Microsoft/AirSim.git
WORKDIR /$FOLDER_NAME/AirSim
RUN ./setup.sh
RUN ./build.sh
# use ./build.sh --debug to build in debug mode
# Create ~Documents/AirSim folder
WORKDIR /home/$USERNAME/Documents/AirSim

# =======
WORKDIR /$FOLDER_NAME


# ============================== 
# Setup TartanAir Data Pipeline
# ==============================

# ============================== 
# Pull TartanAir Data Pipeline Code (https://github.com/Amigoshan/tartanair.git)
# ==============================
ARG WORKSPACE=catkin_ws
WORKDIR /$FOLDER_NAME/$WORKSPACE
RUN sudo chown -R $USERNAME /$FOLDER_NAME/$WORKSPACE
WORKDIR /$FOLDER_NAME/$WORKSPACE/src
# Copy tartanair/ from host into docker image
COPY ./tartanair/ /$FOLDER_NAME/$WORKSPACE/src


# ============================== 
# Catkin build TartanAir ROS workspace
# ==============================
WORKDIR /$FOLDER_NAME/$WORKSPACE
RUN source /opt/ros/melodic/setup.bash \
&& sudo -E catkin build

# Source /$FOLDER_NAME/$WORKSPACE/devel/setup.bash automatically
RUN echo "source /$FOLDER_NAME/$WORKSPACE/devel/setup.bash" >> ~/.bashrc



# ============================== 
# Install Additional Packages
# ==============================

# ============================== 
# Dependencies For TartanAir Mapping
# ==============================
RUN sudo apt-get update && sudo apt-get install python-tk python-numba -y --no-install-recommends
RUN pip install pip wheel msgpack-rpc-python pyquaternion scipy networkx
RUN sudo apt-get clean autoremove


# ============================== 
# Other Utilities
# ============================== 
# Text Editor and Vim
RUN sudo apt-get update && sudo apt-get install gedit vim -y --no-install-recommends
# Image Viewer
RUN sudo apt-get update && sudo apt-get install eog -y --no-install-recommends
# Clean Up
RUN sudo apt-get clean autoremove



# ============================== 
# HW Accelerate
# ============================== 
# run xhost +local:docker on your host machine if any issues opening UI
# test with cmd: roscore & rviz
# enable NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#dockerfiles)
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV SDL_VIDEODRIVER=offscreen
ENV SDL_HINT_CUDA_DEVICE=0
ENV QT_X11_NO_MITSHM=1



# =================================== 
# Set Up Working Directories for Container
# =================================== 
## Create directories to store output data
# Create directory to store maps
WORKDIR /$FOLDER_NAME/data/map_dir/OccMap
# Create directory to store graphs
WORKDIR /$FOLDER_NAME/data/graph_dir/OccMap
# Create directory to store paths
WORKDIR /$FOLDER_NAME/data/path_dir
# Create directory to store data
WORKDIR /$FOLDER_NAME/data/data_dir

## Create directories for data verification
# TBD

# Permissions for above directories
RUN sudo chown -R $USERNAME /$FOLDER_NAME/data


# Set working directory for container and END
WORKDIR /$FOLDER_NAME
