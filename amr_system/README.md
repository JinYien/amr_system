# amr_system

全体の起動と、操作指令を統合するコマンドノード、車輪の静的 TF を提供する

## 動作モードと操作権限

- manual: ジョイスティック指令を順運動学で車輪速度に変換して直接出力する
- auto + user: ハンドルにかかる力を整形して車輪速度を生成する
- auto + robot: 自律計画ノードの車輪速度をそのまま出力する
- auto + shared: TODO

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | /command/joystick | Command |
| 購読 | /command/robot | Command |
| 購読 | /robot/control | Control |
| 購読 | /robot/teensy | Teensy |
| 配信 | /robot/command | Command |

## 設定可能パラメータ

### config/command.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| topics.command_publisher | 最終的な移動指令の出力トピック |  | /robot/command |
| topics.control_subscriber | 制御モードの入力トピック |  | /robot/control |
| topics.joystick_subscriber | ジョイスティック指令の入力トピック |  | /command/joystick |
| topics.robot_subscriber | 自律移動指令の入力トピック |  | /command/robot |
| topics.teensy_subscriber | Teensy テレメトリの入力トピック |  | /robot/teensy |
| loop.period | 制御周期 [s] | 小さくすると応答は速くなるが、CPU 負荷が増える | 0.05 |
| force.deadzone | 力入力の不感帯 [N] | 大きくすると小さな力を無視し、誤動作を減らせる | 1.0 |
| force.clamp_min | 力入力の最小値 [N] | 入力が小さすぎる場合の下限を決める | -3.0 |
| force.clamp_max | 力入力の最大値 [N] | 入力が大きすぎる場合の上限を決める | 3.0 |
| force.useful_range | 有効な力入力範囲 [N] | 小さくすると、少ない力で大きな速度指令が出る | 3.0 |
| force.low_pass_alpha | 力入力の平滑化係数 | 小さくすると滑らかになるが、反応は遅くなる | 0.35 |
| linear.forward_gain | 前進速度ゲイン [deg/s] | 大きくすると、同じ力入力でも前進速度が速くなる | 25.0 |
| linear.reverse_gain | 後進速度ゲイン [deg/s] | 大きくすると、同じ力入力でも後進速度が速くなる | 17.0 |
| linear.max | 前進出力の上限 [deg/s] | 前進方向の最大速度を制限する | 25.0 |
| linear.min | 後進出力の下限 [deg/s] | 後進方向の最大速度を制限する | -17.0 |
| linear.max_acceleration | 前進・後進の加速制限 [deg/s²] | 大きくすると加速は速くなるが、動きが急になりやすい | 6.0 |
| linear.max_deceleration | 前進・後進の減速制限 [deg/s²] | 大きくすると素早く減速・停止できる | 12.0 |
| angular.max_acceleration | 旋回加速制限 [deg/s²] | 大きくすると旋回応答は速くなるが、動きが急になりやすい | 90.0 |

### config/lidar.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| frame_id | 出力スキャンのフレーム ID |  | lidar |
| lidar_topic | LaserScan の出力トピック |  | robot/lidar |
| imu_topic | IMU データの出力トピック |  | robot/imu |
| publish_freq | スキャン配信周波数 [Hz] | 高いほど更新は速くなるが、CPU 負荷も増える | 30.0 |
| xfer_format | LiDAR データの出力形式 | 0 は PointCloud2、1 は LaserScan として出力する | 1 |
| range_min | 有効距離の下限 [m] | これより近い点を無効として除外する | 0.05 |
| range_max | 有効距離の上限 [m] | これより遠い点を無効として除外する | 5.0 |
| min_z | 使用する点群の高さ下限 [m] | 床面など低すぎる点を除外する | -0.30 |
| max_z | 使用する点群の高さ上限 [m] | 天井や高すぎる点を除外する | 1.20 |
| scan_angle_offset | LaserScan の角度補正 [rad] | LiDAR の取付向きをロボット前方基準に合わせる | -1.5707963 |
| scan_angle_min | スキャン角度の下限 [rad] | LaserScan として出力する角度範囲の開始位置を決める | -3.1415 |
| scan_angle_max | スキャン角度の上限 [rad] | LaserScan として出力する角度範囲の終了位置を決める | 3.1416 |
| scan_angle_increment | スキャン角度の分解能 [rad] | 小さいほど細かくなるが、データ量が増える | 0.0087 |
| scan_time | 1 スキャンの周期 [s] | publish_freq と整合する値にする | 0.0333 |
| use_inf | 無効点の表現方法 | true は inf、false は range_max として出力する | true |