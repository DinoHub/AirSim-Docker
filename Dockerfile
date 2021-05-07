# docker build -t ue4:4.18 .
# xhost +local:docker && docker run --rm -it -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/data/projects/cinematography:/workspace/volume" -e "DISPLAY=${DISPLAY}" --ipc="host" ue4
# docker commit <CONTAINER_ID> ue4:latest after first run of ue to prevent long ue init each time from recompiling shader maps

FROM osrf/ros:melodic-desktop-full
FROM nvcr.io/nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
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

# ## User Setup
# # Add a user with the same user_id as the user outside the container
# # Requires a docker build argument `user_id`
# ARG USERID=1000
# ARG GROUPID=1000
# ENV USERNAME ue4
# RUN groupadd -g $GROUPID -o $USERNAME
# RUN useradd -m --uid $USERID --gid $GROUPID -o -s /bin/bash $USERNAME \
# RUN echo "$USERNAME:$USERNAME" | chpasswd \
# && adduser $USERNAME sudo \
# && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME
# WORKDIR /home/$USERNAME
# RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME
# USER $USERNAME
# CMD /bin/bash

# # Enable PulseAudio support
# RUN sudo apt-get install pulseaudio-utils -y --no-install-recommends
# COPY --chown=$USERNAME:$USERNAME pulseaudio-client.conf /etc/pulse/client.conf

## ROS Installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install ros-melodic-desktop-full  -y --no-install-recommends
RUN sudo apt-get install ros-melodic-catkin ros-melodic-teleop-twist-keyboard python-pip python-wstool python-catkin-tools -y --no-install-recommends
RUN sudo apt-get install curl gnupg2 libpcap-dev libcgal-dev libcgal-demo libeigen3-dev openssh-server -y --no-install-recommends

# ## AirSim Installation
# RUN sudo apt-get install python3 python3-pip python3-dev sudo libglu1-mesa-dev xdg-user-dirs pulseaudio -y --no-install-recommends
# RUN sudo apt-get install build-essential cmake cppcheck gdb git vim wget tmux less htop python python-pip python-tk -y --no-install-recommends

# Cleanup
RUN sudo apt-get clean autoremove

# ## Python
# RUN pip3 install setuptools wheel --no-cache-dir
# RUN pip3 install 'ue4cli>=0.0.41' 'conan-ue4cli>=0.0.21' ue4-ci-helpers --no-cache-dir
# RUN pip3 install msgpack-rpc-python numpy airsim --no-cache-dir
# RUN pip install colored-traceback catkin_tools msgpack-rpc-python torch future --no-cache-dir

# ### Copy Files into Docker
# ARG FOLDER_NAME=workspace

# ## `filming_meta` README Setup
# WORKDIR /$FOLDER_NAME
# RUN sudo chown -R $USERNAME:$USERNAME /$FOLDER_NAME
# #Assuming wstool init src src/filming_meta/main.rosinstall has been done on your host due to bitbucket permissions
# COPY --chown=$USERNAME:$USERNAME src /$FOLDER_NAME/src

# RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN echo "alias c='catkin build --this'" >> ~/.bashrc
# RUN echo "alias s='source ~/.bashrc'" >> ~/.bashrc
# RUN echo "alias ue='/$FOLDER_NAME/UnrealEngine/Engine/Binaries/Linux/UE4Editor'" >> ~/.bashrc
# RUN echo "alias takeoff='rosservice call /sm/takeoffSrv'" >> ~/.bashrc

# WORKDIR src/dji_osdk/build
# RUN sudo cmake ..
# RUN sudo make install
# WORKDIR /$FOLDER_NAME
# RUN source /opt/ros/melodic/setup.bash && sudo -E catkin build -j8
# RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc
# WORKDIR src/filming_kf
# RUN pip install --user .

## Unreal Setup (https://bitbucket.org/castacks/cluster/wiki/airsim_ros)
# ( https://microsoft.github.io/AirSim/build_linux/)
WORKDIR /$FOLDER_NAME

RUN git clone -b 4.25 https://theairlab:airlab123@github.com/EpicGames/UnrealEngine.git
WORKDIR UnrealEngine
RUN sudo chown -R $USERNAME /home/$USERNAME
# RUN sudo chown -R $USERNAME /$FOLDER_NAME
RUN ./Setup.sh
RUN ./GenerateProjectFiles.sh
RUN make
WORKDIR /$FOLDER_NAME

## AirSim Setup (https://microsoft.github.io/AirSim/build_linux/)
RUN git clone https://github.com/Microsoft/AirSim.git
WORKDIR airsim
RUN ./setup.sh
RUN ./build.sh
# use ./build.sh --debug to build in debug mode

# Copy Unreal Environments (Forest, Gascola ...)
# https://cmu.box.com/s/ewia8tusjc9iqxnc5iimr7wuug9gvs3h (Would you like to use car simulation? Choose no to use quadrotor simulation. Error at startup: VehicleSetting for vehicle name BP_FlyingPawn_2413 was requested but not found)
# https://cmu.box.com/s/lknouwj7w9fhfnxcj3taqe3su0ns28ww (Unable to read project status.)
# COPY --chown=$USERNAME environment /$FOLDER_NAME/environment

## HW Accelerate
# run xhost +local:docker on your host machine if any issues opening UI
# test with cmd: roscore & rosrun rviz rviz
# enable NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#dockerfiles)
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV SDL_VIDEODRIVER=offscreen
ENV SDL_HINT_CUDA_DEVICE=0
ENV QT_X11_NO_MITSHM=1
