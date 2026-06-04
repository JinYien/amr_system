# amr_message

共有するメッセージ定義と QoS プロファイルを提供する

## メッセージ型

| メッセージ | 用途 |
| --- | --- |
| Command | 移動指令（車体速度・車輪速度・ハンドル動作） |
| Control | 制御モード・操作権限の切り替え |
| State | オドメトリから推定したロボットの状態 |
| Teensy | Teensy の生テレメトリ |
| PidCommand<br>PidGain | PID チューニング用の速度指令 / ゲイン |
| Detection<br>DetectionArray | 物体検出の結果 |
| Intersection | 交差点の通行可能方向と隙間情報 |

## 設定可能パラメータ

### config/qos.yaml

| 項目 | 内容 | 影響 | 初期値 |
| --- | --- | --- | --- |
| reliability | 通信の信頼性 | reliable：再送によりデータ欠落を減らせるが、遅延や通信負荷が増える<br>best_effort：再送しないため低遅延だが、データが欠落する場合がある | best_effort |
| durability | データの保持方法 | transient_local：後から起動した購読ノードも直近のデータを受信できる<br>volatile：データを保持せず、購読開始後のデータのみ受信する | volatile |
| history | メッセージ履歴の管理方法 | keep_all：可能な限りすべての履歴を保持する<br>keep_last：直近の depth 件のみ保持する | keep_last |
| depth | 保持するメッセージ数 | 大きくすると一時的な処理遅れに強くなるが、メモリ使用量や遅延が増える | 1 |