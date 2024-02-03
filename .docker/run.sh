#!/bin/bash

DOCKER_IMAGE="tang_lidar:latest"
CONTAINER_NAME="tang_lidar"

docker run --rm -it \
    --env="DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --volume="/home/hashimoto/ros2_ws:/root/ros2_ws" \
    --name=$CONTAINER_NAME \
    --privileged \
    --net=host \
    $DOCKER_IMAGE \
    bash