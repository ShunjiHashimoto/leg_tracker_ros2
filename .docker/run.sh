#!/bin/bash

DOCKER_IMAGE="leg_tracker:latest"
CONTAINER_NAME="leg_tracker_ros2"

docker run --rm -it \
    --env="DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --volume="/home/hashimoto/ros2_ws:/root/ros2_ws" \
    --name=$CONTAINER_NAME \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash