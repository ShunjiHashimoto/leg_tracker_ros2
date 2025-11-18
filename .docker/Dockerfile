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
  terminator \
  libfreetype6-dev \
  libpng-dev \
  iputils-ping \
  net-tools

# Install python packages
RUN apt-get install -y \
  python3-pykdl \
  python3-pip \
  python3-libgpiod

# Install ROS packages
RUN apt-get update && apt-get install -y \
  ros-humble-urg-node \
  ros-humble-sensor-msgs \
  ros-humble-rviz2 \
  ros-humble-rmw-cyclonedds-cpp && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# Install pip packages
# pip install numpy==1.24.4
RUN pip install --no-cache-dir \
  pyserial==3.5 \
  pykalman==0.9.7 \
  spidev==3.6 \
  rpi-lgpio==0.6 \
  gpiozero==2.0.1 \
  transforms3d==0.4.2 \
  matplotlib==3.9.0

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
RUN echo 'alias start="ros2 launch leg_tracker_ros2 leg_tracker.launch.py"' >> ~/.bashrc
RUN echo 'alias tang="ros2 run tang_control tang_control"' >> ~/.bashrc
RUN echo 'alias leg="cd ~/ros2_ws/src/leg_tracker_ros2"' >> ~/.bashrc
