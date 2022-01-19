xhost +local:docker 
docker run --rm -it \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e "DISPLAY=${DISPLAY}" \
    --gpus=all \
    --ipc="host" \
    --name tartanair \
    tartanairsim1.4:base

# Include additional argument -v "/path/to/your/UE_env:/workspace/UnrealProj" to mount a local UE environment inside the container at /workspace/UnrealProj