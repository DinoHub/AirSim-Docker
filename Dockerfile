### Instructions for Docker
## build image from this dockerfile. E.g. IMAGE_NAME = ue4, IMAGE_TAG = Tartan
# docker build -t <IMAGE_NAME>:<IMAGE_TAG> .

## run a container from the built image.
# xhost +local:docker && docker run --rm -it -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/path/to/your/UE_env:/workspace/UnrealProj" -e "DISPLAY=${DISPLAY}" --ipc="host" <IMAGE_NAME>:<IMAGE_TAG>
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

# Replace with local SG mirrors
RUN sed --in-place --regexp-extended "s/(\/\/)(archive\.ubuntu)/\1sg.\2/" /etc/apt/sources.list
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ Asia/Singapore
RUN apt-get update
RUN apt-get install -y --no-install-recommends sudo curl tzdata
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && sudo dpkg-reconfigure -f noninteractive tzdata

## UI Support
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


## User Setup
# Add a user with the same user_id as the usewhen running the sample-graph code. Is this due to the unsuccessful roslaunch roadmap_generator_blocks.launch? Or is there a missing package?r outside the container
# Requires a docker build argument `USERID` >> docker build --build-arg USERID=youruserid
ARG USERID=1000
ARG GROUPID=1000
# Use 'ue4' for username and group name
ENV USERNAME=ue4
# RUN groupadd --gid $GROUPID $USERNAME 
# RUN useradd --uid $USERID --gid $GROUPID --create-home --shell /bin/bash $USERNAME \
RUN echo "$USERNAME:$USERNAME" | chpasswd \
&& adduser $USERNAME sudo \
&& echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME
WORKDIR /home/$USERNAME
RUN sudo chown --recursive $USERNAME:$USERNAME /home/$USERNAME
USER $USERNAME

### INSTALLS!
ARG FOLDER_NAME=workspace
WORKDIR /$FOLDER_NAME
RUN sudo chown -R $USERNAME:$USERNAME /$FOLDER_NAME

## Unreal Engine (https://bitbucket.org/castacks/cluster/wiki/airsim_ros)
# (https://microsoft.github.io/AirSim/build_linux/)
RUN git clone -b 4.25 https://theairlab:airlab123@github.com/EpicGames/UnrealEngine.git

# RUN sudo chown -R $USERNAME /home/$USERNAME
WORKDIR /$FOLDER_NAME/UnrealEngine
RUN ./Setup.sh
RUN ./GenerateProjectFiles.sh
USER $USERNAME
RUN make
WORKDIR /$FOLDER_NAME


## ROS Melodic
WORKDIR /home/$USERNAME 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

RUN sudo apt-get update && sudo apt-get install ros-melodic-desktop-full  -y --no-install-recommends
# RUN sudo apt-get update && sudo apt-get install ros-melodic-desktop-full ros-melodic-perception ros-melodic-simulators ros-melodic-urdf-sim-tutorial ros-melodic-perception-pcl ros-melodic-gazebo-ros-pkgs -y --no-install-recommends
# dependencies from Wenshan Doc
RUN sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server ros-melodic-dynamic-edt-3d -y --no-install-recommends
# # dependencies for building packages (TODO: necessary?)
RUN sudo apt-get install ros-melodic-catkin ros-melodic-teleop-twist-keyboard python-pip python-wstool python-catkin-tools -y --no-install-recommends
# RUN sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y --no-install recommends

# Cleanup
RUN sudo apt-get clean autoremove


## AirSim (https://microsoft.github.io/AirSim/build_linux/)
WORKDIR /$FOLDER_NAME
# v1.4 is working. Stick to v1.4 first.
RUN git clone -b v1.4.0-linux https://github.com/Microsoft/AirSim.git
# # latest AirSim
# RUN git clone https://github.com/Microsoft/AirSim.git

WORKDIR AirSim
RUN ./setup.sh
RUN ./build.sh
# use ./build.sh --debug to build in debug mode


### Setup Tartanair Data Pipeline

## Pull Tartanair Data Pipeline Code (https://github.com/Amigoshan/tartanair.git)
ARG WORKSPACE=catkin_ws
WORKDIR /$FOLDER_NAME/$WORKSPACE
RUN sudo chown -R $USERNAME /$FOLDER_NAME/$WORKSPACE
WORKDIR src
COPY ./tartanair/ /$FOLDER_NAME/$WORKSPACE/src

# (Git clone doesn't work somehow)
# RUN git clone https://theairlab:airlab123@github.com/Amigoshan/tartanair.git
# RUN mv tartainair src

## Catkin build tartanair ROS workspace
WORKDIR /$FOLDER_NAME/$WORKSPACE
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source /opt/ros/melodic/setup.bash \
&& sudo -E catkin build
# data_type expo_base frontier_base octomap_mapping ompl planner_base roadmap_generator sample_pipeline

# Source /$FOLDER_NAME/$WORKSPACE/devel/setup.bash automatically
RUN echo "source /$FOLDER_NAME/$WORKSPACE/devel/setup.bash" >> ~/.bashrc

# Install Mapping Dependencies
RUN sudo apt-get update && sudo apt-get install python-tk python-numba -y --no-install-recommends
RUN pip install msgpack-rpc-python pyquaternion scipy networkx

## Create directories to store output data
# Create directory to store maps
WORKDIR /$FOLDER_NAME/data/map_dir/OccMap
# Create directory to store graphs
WORKDIR /$FOLDER_NAME/data/graph_dir/OccMap
# Create directory to store paths
WORKDIR /$FOLDER_NAME/data/path_dir
# Create directory to store data
WORKDIR /$FOLDER_NAME/data/data_dir
# Permissions for above directories
RUN sudo chown -R $USERNAME /$FOLDER_NAME/data

## Create directories for data verification
# TBD


### Install Others
# Text Editors
RUN sudo apt-get update && sudo apt-get install gedit vim -y --no-install-recommends
# Image Viewer
RUN sudo apt-get update && sudo apt-get install eog -y --no-install-recommends


### HW Accelerate
# run xhost +local:docker on your host machine if any issues opening UI
# test with cmd: roscore & rosrun rviz rviz
# enable NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#dockerfiles)
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV SDL_VIDEODRIVER=offscreen
ENV SDL_HINT_CUDA_DEVICE=0
ENV QT_X11_NO_MITSHM=1


### Set working directory for container and END
WORKDIR /$FOLDER_NAME