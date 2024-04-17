FROM arm64v8/ros:humble-perception
# FROM osrf/ros:humble-desktop-full

# キーボード設定の選択を省略
ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
  build-essential \
  gdb \
  cmake \
  git \
  vim \
  wget \
  tmux \
  x11-apps \
  terminator 

# Install python packages
RUN apt-get install -y \
  python3-pykdl \
  python3-pip \
  python3-gpiozero

# Install ROS packages
RUN apt-get update && apt-get install -y \
  ros-humble-urg-node \
  ros-humble-sensor-msgs \
  ros-humble-rviz2 && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pip packages
RUN pip install \
  pykalman \
  spidev 

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

RUN echo 'source ~/ros2_ws/before_bringup.sh' >> ~/.bashrc
RUN echo 'alias cm="cd ~/ros2_ws && colcon build --symlink-install --packages-select leg_tracker_ros2"' >> ~/.bashrc