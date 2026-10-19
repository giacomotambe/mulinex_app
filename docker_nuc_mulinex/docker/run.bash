#!/bin/bash

# Create /tmp/.docker.xauth if it does not already exist.
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

IMAGE_NAME=mulinex
IMAGE_TAG=humble

xhost +
docker run \
    `# Share the host’s network stack and interfaces. Allows multiple containers to interact with each other.` \
    --net=host \
    --ipc=host \
    `# Interactive processes, like a shell.` \
    -it \
    `# Clean up the container after exit.` \
    --rm \
    `# Use GUI.` \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/dev/dri:/dev/dri" \
    -v /dev/input:/dev/input \
    `# Privileged mode for device access.` \
    --privileged \
    `# ROS 2 settings.` \
    --env="ROS_DOMAIN_ID=10" \
    --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --env="RCUTILS_COLORIZED_OUTPUT=1" \
    `# Mount the folders in this directory.` \
    -v ${PWD}:/home/$USER/mulinex_ws \
    -v ~/docker/${IMAGE_NAME}/Plotjuggler:/home/$USER/.config/PlotJuggler \
    `# Preserve bash history for autocomplete).` \
    --env="HISTFILE=/home/.bash_history" \
    --env="HISTFILESIZE=$HISTFILESIZE" \
    -v ~/.bash_history:/home/.bash_history \
    `# Audio support in Docker.` \
    --device /dev/snd \
    -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
    --group-add $(getent group audio | cut -d: -f3) \
    `# Enable SharedMemory between host and container.` \
    `# https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/` \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    `# Mount folders useful for VS Code.` \
    -v /home/$USER/.vscode:/home/$USER/.vscode \
    -v /home/$USER/.vscode-server:/home/$USER/.vscode-server \
    -v /home/$USER/.config/Code:/home/$USER/.config/Code \
    `# Matplotlib environment variable.` \
    --env="MPLCONFIGDIR=/home/$USER/.matplotlib" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --name mulinex \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash
