# AirSim-Docker

## Set up Unreal Project 
(https://microsoft.github.io/AirSim/unreal_custenv/)
### In Windows Machine
1. Create new project with custom environment of choice. Copy Unreal project folder to Linux machine

### In Linux Machine
1. From AirSim/Unreal/, copy Plugins/ into the Unreal project folder.


## Docker Pre-Reqs
1. docker installed on host (https://docs.docker.com/engine/install/ubuntu/)
    - Optional for QOL: remove need for sudo for docker (https://docs.docker.com/engine/install/linux-postinstall/)
1. nvidia-docker installed on host (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian). 
1. \> 120GB in directory where docker images are built. This is typically in '/', recommend to shift to '/home' or other directory with larger storage (https://www.guguweb.com/2019/02/07/how-to-move-docker-data-directory-to-another-location-on-ubuntu/)
1. git clone https://github.com/Amigoshan/tartanair into same directory as dockerfile. Rename as /tartanair/
1. Ensure docker image directory has "defaults" options or NO "nosuid" option.

## Build Docker Image
1. `cd /directory/containing/dockerfile/`
1. `docker build -t <IMAGE_NAME>:<IMAGE_TAG> .`
    - E.g. IMAGE_NAME = ue4, IMAGE_TAG = Tartan will build a docker image ue4:Tartan
    - Use `docker container ls -a` to view the built/incomplete image.

## Run Docker Container from built Image
1. ```xhost +local:docker && docker run --rm -it -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/path/to/your/UE_env:/workspace/UnrealProj" -e "DISPLAY=${DISPLAY}" --ipc="host" <IMAGE_NAME>:<IMAGE_TAG>```
    - Enables GUI display on host
    - Mounts a directory from host into the docker container
    - More info here: https://docs.docker.com/engine/reference/commandline/run/

## Commit Container Changes to Container as new Image
1. Find the CONTAINER_NAME using `docker container ls`
1. `docker commit <CONTAINER_NAME> <IMAGE_NAME>:<IMAGE_TAG>`

## Launch Additional Terminal for Running Container
1. Without GUI support
    - `docker exec -it <CONTAINER_NAME> bash`
1. With GUI support
    - `xhost +local:docker && docker exec -it -e "DISPLAY=${DISPLAY}" <CONTAINER_NAME> bash`

## Launch Unreal Engine inside Docker Container
1. Copy AirSim plugins to Unreal Project.
    - `cp /workspace/AirSim/Unreal/Plugins/ /path/to/Unreal/Project/`
1. `./path/to/UnrealEngine/Engine/Binaries/Linux/UE4Editor /path/to/Unreal/Project/<PROJECT_NAME>.uproject`
1. Unreal Engine will prompt to rebuild AirSim. Click "Yes". First compile will take awhile.