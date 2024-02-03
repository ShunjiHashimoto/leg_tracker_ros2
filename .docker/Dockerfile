FROM osrf/ros:humble-desktop-full

# キーボード設定の選択を省略
ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
git \
vim \
wget \
tmux \
terminator && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
  ros-humble-urg-node \
  ros-humble-sensor-msgs && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

# entrypoint.sh スクリプトをコンテナにコピー
COPY entrypoint.sh /entrypoint.sh

# スクリプトを実行可能にする
RUN chmod +x /entrypoint.sh

# スクリプトをエントリーポイントとして設定
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Create ROS workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src