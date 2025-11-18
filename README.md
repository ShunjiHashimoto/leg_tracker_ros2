# leg_tracker_ros2

LiDARを使って脚検出を行い、カルマンフィルタを用いてトラッキングを行うパッケージです。  

このパッケージは[angusleigh/leg_tracker](https://github.com/angusleigh/leg_tracker)をベースにROS 2対応を行いました。ROS 1版で実装されていたカルマンフィルタベースの検出・追跡ロジックを引き継ぎつつ、ROS 2に適合するようにビルドシステムやメッセージ定義を移植しています。


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

## ライセンス

本パッケージはBSD 3-Clause Licenseで配布されています。ライセンス全文および原著作者のクレジットは`LICENSE`を参照してください。
