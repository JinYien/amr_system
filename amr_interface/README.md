# amr_interface

ブラウザから AMR を遠隔操作・監視するインターフェース

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | /robot/state | State |
| 配信 | /command/joystick | Command |
| 配信 | /robot/control | Control |

## 設定可能パラメータ

### config/interface.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| topics.command_publisher | ジョイスティック指令の出力トピック |  | /command/joystick |
| topics.control_publisher | 制御モードの出力トピック |  | /robot/control |
| topics.state_subscriber | ロボット状態の入力トピック |  | /robot/state |
| rates.command_publish | 指令送信周期 [Hz] | 大きくすると操作は滑らかになるが、CPU 負荷や通信量が増える | 20 |
| http.host | Web サーバの待ち受けアドレス | 0.0.0.0 は全ネットワークから接続可能、特定 IP にすると接続元を制限できる | 0.0.0.0 |
| http.port | Web サーバのポート番号 | 他サービスとポートが重なる場合に変更する | 8080 |
| limits.max_linear_velocity | 最大並進速度 [m/s] | 大きくすると前後方向に速く動けるが、操作が急になりやすい | 1.0 |
| limits.max_angular_velocity | 最大旋回速度 [rad/s] | 大きくすると素早く旋回できるが、動きが不安定になりやすい | 3.0 |
| vocabulary.modes | 使用可能な制御モード | リストにない制御モードは無視される | ["manual", "auto"] |
| vocabulary.authorities | 使用可能な操作権限 | リストにない操作権限は無視される | ["", "user", "robot", "shared"] |
| vocabulary.middle_modes | 使用可能なハンドル動作モード | リストにないハンドル動作モードは無視される | ["", "nudge", "pulse"] |
| vocabulary.middle_actions | 使用可能なハンドル動作方向 | リストにないハンドル動作方向は無視される | ["", "left", "right"] |