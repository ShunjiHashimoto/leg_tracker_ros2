# leg_tracker_ros2

LiDARを使って脚検出を行い、カルマンフィルタを用いてトラッキングを行うパッケージです。  


## デバッグ方法
まずビルドをする  
```bash
$ colcon build --packages-select leg_tracker_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

rviz2を起動する   
```bash
$ rviz2
```

rosbagを実行する    
```bash
$ ros2 bag play -s sqlite3 "/root/ros2_ws/src/leg_tracker_ros2/rosbag/demos/demo_stationary_simple_environment"
```

vscodeのデバッガを実行する  