# amr_tuner

車輪モータの速度制御 PID を Ziegler-Nichols 法で調整する GUI ツール

## 使い方

1. Kp を入力して Set PID で適用し、速度を設定して Start Motor
2. Kp を少しずつ上げ、持続振動（一定振幅の振動）が出る点を探す
3. Calculate PID で、その時の Kp（限界感度）と測定周期から各方式の PID ゲインが表示される

## トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| 購読 | /tuner/teensy | Teensy |
| 配信 | /tuner/command | PidCommand |
| 配信 | /tuner/gain | PidGain |

## 設定可能パラメータ

### config/tuner.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| topics.gain | PID ゲイン送信用トピック |  | /tuner/gain |
| topics.command | 速度指令送信用トピック |  | /tuner/command |
| topics.teensy | モータ速度フィードバック用トピック |  | /tuner/teensy |
| rates.serial_read | Teensy からの受信周期 [Hz] | 高くすると受信遅延は減るが、CPU 負荷が増える | 1000 |
| rates.drive_command | 駆動指令の送信周期 [Hz] | 高くすると指令更新が滑らかになる | 100 |
| rates.ros_spin_interval | GUI 側の ROS 処理間隔 [ms] | 小さくすると受信は速くなるが、GUI 負荷が増える | 10 |
| serial.identifier | Teensy の USB 識別子 | 不一致だと接続に失敗する | VID:PID=16C0:0483 |
| serial.baudrate | シリアル通信のボーレート |  | 115200 |
| serial.timeout | シリアル読み取りタイムアウト [s] | 大きくすると受信待ち時間が長くなる | 0.01 |
| serial.boot_delay | 接続後にファームウェア起動を待つ時間 [s] | 短すぎると初期化指令を取りこぼす場合がある | 1.0 |
| serial.gain_apply_delay | PID ゲイン適用後の待ち時間 [s] | 短すぎるとゲイン設定が反映されない場合がある | 0.5 |
| window.width | ウィンドウ幅 [px] | GUI の横幅が変わる | 1400 |
| window.height | ウィンドウ高さ [px] | GUI の縦幅が変わる | 950 |
| window.x | 起動時の表示位置 X [px] | ウィンドウの初期表示位置を左右に動かす | 100 |
| window.y | 起動時の表示位置 Y [px] | ウィンドウの初期表示位置を上下に動かす | 100 |
| window.left_panel | 左パネルの幅比率 [%] | グラフや操作領域の表示幅を調整する | 75 |
| window.right_panel | 右パネルの幅比率 [%] | 設定・入力領域の表示幅を調整する | 35 |
| window.gui_update_interval | GUI 再描画間隔 [ms] | 小さくすると表示は滑らかになるが、CPU 負荷が増える | 1 |
| pid_gain.min | PID ゲイン入力の最小値 | 入力できる PID ゲインの下限を決める | 0.0 |
| pid_gain.max | PID ゲイン入力の最大値 | 入力できる PID ゲインの上限を決める | 500.0 |
| pid_gain.step | PID ゲイン入力の刻み | 小さくすると細かく調整できる | 0.001 |
| pid_gain.decimals | PID ゲインの表示桁数 | 表示する小数点以下の桁数を変える | 5 |
| velocity.min | 速度指令の最小値 [deg/s] | 指令できる回転速度の下限を決める | -1000.0 |
| velocity.max | 速度指令の最大値 [deg/s] | 指令できる回転速度の上限を決める | 1000.0 |
| velocity.slider_scale | 速度スライダーの倍率 | スライダー操作時の分解能を調整する | 1 |
| velocity.slider_tick_interval | 速度スライダーの目盛り間隔 | スライダーに表示する目盛り間隔を変える | 200 |
| velocity.step | 速度入力の刻み | 小さくすると速度を細かく入力できる | 1.0 |
| velocity.decimals | 速度入力の表示桁数 | 表示する小数点以下の桁数を変える | 2 |
| wheel.min_radius | 車輪半径入力の最小値 [m] | 速度換算に使う車輪半径の下限を決める | 0.001 |
| wheel.max_radius | 車輪半径入力の最大値 [m] | 速度換算に使う車輪半径の上限を決める | 1.0 |
| wheel.radius_step | 車輪半径入力の刻み | 小さくすると半径を細かく入力できる | 0.001 |
| wheel.radius_decimals | 車輪半径の表示桁数 | 表示する小数点以下の桁数を変える | 3 |
| wheel.default_radius | 車輪半径の初期値 [m] | 速度換算パネルの初期値として使われる | 0.108 |
| linear_velocity.min | 線速度入力の最小値 [m/s] | 換算できる線速度の下限を決める | -100.0 |
| linear_velocity.max | 線速度入力の最大値 [m/s] | 換算できる線速度の上限を決める | 100.0 |
| linear_velocity.step | 線速度入力の刻み | 小さくすると線速度を細かく入力できる | 0.1 |
| linear_velocity.decimals | 線速度の表示桁数 | 表示する小数点以下の桁数を変える | 2 |
| graph.decimal_precision | グラフ表示値の丸め桁数 | 表示する数値の精度を変える | 3 |
| graph.min_time_window | 表示時間窓の最小値 [s] | グラフの表示範囲として選べる最小値を決める | 0.0 |
| graph.max_time_window | 表示時間窓の最大値 [s] | グラフの表示範囲として選べる最大値を決める | 300.0 |
| graph.time_window_step | 表示時間窓の刻み [s] | 時間窓を変更するときの刻み幅を決める | 5.0 |
| graph.time_window_decimals | 表示時間窓の表示桁数 | 時間窓入力で表示する小数点以下の桁数を変える | 0 |
| graph.default_time_window | 表示時間窓の初期値 [s] | 起動時のグラフ表示範囲を決める、0 で自動スケール | 0.0 |
| graph.amplitude_sample_size | 振幅計算に使うサンプル数 | 大きくすると振幅計算は安定するが、変化への反応は遅くなる | 100 |
| analysis.min_valid_period | 有効周期の最小値 [s] | これより短い周期は異常値として無視する | 0.1 |
| analysis.max_valid_period | 有効周期の最大値 [s] | これより長い周期は異常値として無視する | 10.0 |
| analysis.peak_detection_window | ピーク検出の窓サイズ | 大きくするとノイズに強くなるが、ピーク検出は鈍くなる | 5 |
| analysis.min_oscillation_periods | PID 算出に必要な最小周期数 | 大きくすると PID 算出は慎重になる | 2 |
| analysis.average_period_window | 平均する周期数 | 大きくすると周期推定は安定するが、変化への反応は遅くなる | 5 |