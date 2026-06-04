# amr_teensy

Teensy 4.0 マイコンとシリアル通信する

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | /robot/command | Command |
| 配信 | /robot/odometry | Odometry |
| 配信 | /robot/state | State |
| 配信 | /robot/teensy | Teensy |

## 設定可能パラメータ

### config/teensy.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| topics.teensy_publisher | Teensy から受信した生データの出力トピック |  | /robot/teensy |
| topics.state_publisher | ロボット状態の出力トピック |  | /robot/state |
| topics.odometry_publisher | オドメトリの出力トピック |  | /robot/odometry |
| topics.command_subscriber | 移動指令の入力トピック |  | /robot/command |
| frames.odometry | オドメトリの親フレーム ID |  | odom |
| frames.base | ロボット本体の子フレーム ID |  | base_link |
| rates.serial_read | シリアル受信周期 [Hz] | 高くすると遅延は減るが、CPU 負荷が増える | 1000 |
| rates.drive_command | 駆動指令の送信周期 [Hz] | 高くすると速度追従は滑らかになるが、通信負荷が増える | 100 |
| rates.handle_command | ハンドル指令の送信周期 [Hz] | 高くするとハンドルのトルク変化が滑らかになる | 20 |
| serial.identifier | 接続する Teensy の USB 識別子 | 使用する Teensy やポートに合わせる。一致しないと接続できない | VID:PID=16C0:0483 |
| serial.baudrate | シリアル通信速度 | Teensy 側の設定と一致させる。不一致だと通信できない | 115200 |
| serial.timeout | シリアル読み取りの待ち時間 [s] | 大きくすると受信待ちが長くなり、処理が遅れる場合がある | 0.01 |
| serial.boot_delay | 接続後にファームウェア起動を待つ時間 [s] | 短すぎると初期化指令を取りこぼす場合がある | 1.0 |
| serial.motor_setup_delay | 各駆動モータ初期化後の待ち時間 [s] | 短すぎると PID 設定が反映されない場合がある | 0.5 |
| drive.left.p_gain | 左駆動モータの P ゲイン | 大きくすると応答は速くなるが、振動しやすくなる | 0.0810 |
| drive.left.i_gain | 左駆動モータの I ゲイン | 大きくすると定常偏差は減るが、オーバーシュートしやすくなる | 0.8005 |
| drive.left.d_gain | 左駆動モータの D ゲイン | 大きくすると振動を抑えやすいが、ノイズに敏感になる | 0.0 |
| drive.right.p_gain | 右駆動モータの P ゲイン | 大きくすると応答は速くなるが、振動しやすくなる | 0.0810 |
| drive.right.i_gain | 右駆動モータの I ゲイン | 大きくすると定常偏差は減るが、オーバーシュートしやすくなる | 0.8005 |
| drive.right.d_gain | 右駆動モータの D ゲイン | 大きくすると振動を抑えやすいが、ノイズに敏感になる | 0.0 |
| handle.torque_limit | ハンドルモータのトルク上限 [Nm] | 大きくすると強い反力を出せるが、安全余裕は小さくなる | 1.0 |
| handle.pulse.torque | pulse 動作の基準トルク [Nm] | 大きくすると pulse の振動が強くなる | 1.0 |
| handle.pulse.hold_time | pulse 各ステップの保持時間 [s] | 大きくすると 1 回の振動がゆっくりになる | 0.05 |
| handle.pulse.sequence | pulse のトルク倍率パターン | 値の並びを変えると、振動の強さや波形が変わる | [1.0, -1.0, 1.0, -1.0, 0.0] |
| handle.nudge.torque | nudge 動作のトルク [Nm] | 大きくすると 1 回の押し出しが強くなる | 1.0 |
| handle.nudge.hold_time | nudge の保持時間 [s] | 大きくすると押し出しが長く続く | 0.10 |
| odometry.max_timestep | オドメトリ積分に使う最大時間刻み [s] | これを超える間隔のデータは、誤差防止のため積分しない | 1.0 |