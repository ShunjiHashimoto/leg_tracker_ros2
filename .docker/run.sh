#!/bin/bash

DOCKER_IMAGE="leg_tracker_ros2:latest"
CONTAINER_NAME="leg_tracker"

docker run --rm -it \
    --env DISPLAY=localhost:11.0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority:ro \
    --volume="${HOME}/ros2_ws:/root/ros2_ws" \
    --volume /dev/shm:/dev/shm \
    --name=$CONTAINER_NAME \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash