# amr_intersection

LiDAR の LaserScan から、前方・左・右それぞれに通行可能な隙間があるかを判定する

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | /robot/lidar | sensor_msgs/LaserScan |
| 配信 | /detection/intersection | amr_message/Intersection |

## 設定可能パラメータ

### config/intersection.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| topics.lidar_subscriber | 入力 LaserScan トピック |  | /robot/lidar |
| topics.intersection_publisher | 交差点判定結果の出力トピック |  | /detection/intersection |
| geometry.safety_margin | 全周の安全余裕 [m] | 大きくすると安全側になるが、ロボット周囲の空き距離は短くなる | 0.05 |
| geometry.min_open_depth | 開放判定に必要な空き距離 [m] | 大きくすると、より奥まで空いている場合のみ「開」と判定する | 1.0 |
| sectors.front_min | 前方判定角度の下限 [deg] | 前方として判定する範囲の開始角度を変える | -50.0 |
| sectors.front_max | 前方判定角度の上限 [deg] | 前方として判定する範囲の終了角度を変える | 50.0 |
| sectors.left_min | 左側判定角度の下限 [deg] | 左側として判定する範囲の開始角度を変える | 60.0 |
| sectors.left_max | 左側判定角度の上限 [deg] | 左側として判定する範囲の終了角度を変える | 120.0 |
| sectors.right_min | 右側判定角度の下限 [deg] | 右側として判定する範囲の開始角度を変える | -120.0 |
| sectors.right_max | 右側判定角度の上限 [deg] | 右側として判定する範囲の終了角度を変える | -60.0 |
| scan.max_skip_beams | 許容する連続無効ビーム数 | 大きくすると欠測に強くなるが、隙間を過大評価しやすくなる | 1 |
| filter.debounce_time | 判定保持時間 [s] | 大きくすると判定は安定するが、反応は遅くなる | 0.0 |