#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# コンテナが実行するコマンドを引き継ぐ
exec "$@"