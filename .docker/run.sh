#!/bin/bash

DOCKER_IMAGE="leg_tracker_ros2:latest"
CONTAINER_NAME="leg_tracker_ros2"

docker run --rm -it \
    --env="DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --volume="${HOME}/ros2_ws:/root/ros2_ws" \
    --name=$CONTAINER_NAME \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash