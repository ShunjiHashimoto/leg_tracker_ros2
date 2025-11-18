# leg_tracker_ros2

LiDARで検出した脚ランドマークをカルマンフィルタでトラッキングし、移動する人物のみを追跡対象として抽出するROS 2パッケージです。ラズパイ5＋HOKUYO UST-10LX＋IMUを前提に、ROS 2対応とIMUの姿勢情報による地図補正を加えて静止障害物を除外する点がオリジナルの[angusleigh/leg_tracker](https://github.com/angusleigh/leg_tracker)からの主な拡張です。

## 特徴
- LiDARによる脚検出＋カルマンフィルタによる人追跡ロジックをROS 2へ移植
- IMUの姿勢補正をかけて動作中のロボットでも地図がズレにくいように補正
- 静止障害物は追跡対象から除外し、移動体のみを追従
- Raspberry Pi 5、Ubuntu 22.04、Hokuyo UST-10LX、[ros2_razor_imu](https://github.com/ShunjiHashimoto/ros2_razor_imu)で動作確認済み

## 必要機器・ソフトウェア
- PC: Raspberry Pi 5（Ubuntu 22.04 LTS 64bit）
- LiDAR: HOKUYO UST-10LX（`urg_node2`でドライバを起動）
- IMU: Razor IMU（`ros2_razor_imu`パッケージを使用）
- その他: Docker (PC上での開発・デバッグ用)、RViz2

## Dockerコンテナ起動

1. ルートディレクトリでビルド
   ```bash
   $ docker build -t leg_tracker_ros2 -f .docker/Dockerfile .
   ```
2. コンテナ起動
   ```bash
   $ .docker/run.sh
   ```

## ビルド
コンテナ内、もしくはUbuntu 22.04環境の`~/ros2_ws`で実行します。

```bash
$ cd ~/ros2_ws
$ colcon build --packages-select leg_tracker_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## 起動手順
1. LiDARドライバ（例）
   ```bash
   $ ros2 run urg_node2 urg_node2_node
   ```
2. IMUドライバ（`ros2_razor_imu`）
   ```bash
   $ ros2 launch ros2_razor_imu razor_imu.launch.py
   ```
3. leg_tracker_ros2
   ```bash
   $ ros2 launch leg_tracker_ros2 leg_tracker.launch.py
   ```

センサーの姿勢・地図補正はIMUトピック（例: `/imu/data`）を購読することで行われ、ロボットが走行中でも地図がずれにくく、同じ人物を継続して識別できます。

## 可視化とデバッグ
- RViz2: `rviz/demo_stationary_simple_environment.rviz`を指定することでLiDARスキャン、検出結果、トラッキング軌跡が一度に確認できます。
  ```bash
  $ rviz2 -d ~/ros2_ws/src/leg_tracker_ros2/rviz/demo_stationary_simple_environment.rviz
  ```
- rosbagを用いた再生
  ```bash
  $ ros2 bag play -s sqlite3 "~/ros2_ws/src/leg_tracker_ros2/rosbag/demos/demo_stationary_simple_environment"
  ```

## ライセンス

本パッケージはBSD 3-Clause Licenseで配布されています。ライセンス全文および原著作者のクレジットは`LICENSE`を参照してください。
