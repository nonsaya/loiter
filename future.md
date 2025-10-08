# 将来計画（Loiter → 自律飛行）

## ステージ0: 現在のベースライン（完了）
- Livox MID360 ドライバ + GLIM（CPU）+ MAVROS + ブリッジ
- 外部オドメトリ → EKF3 に入力（LOITER 安定）
- tmux 一発起動スクリプト（`scripts/launch_loiter_tmux.sh`）

## ステージ1: 無線一発ミッション（アーム → 離陸 → ホバ → 前進 → ホバ → 着陸）
- 目的（無線指示で以下を自動実行）
  1) アーム
  2) 目標高度へ離陸（例: 2.0 m）
  3) 数秒ホバリング
  4) ROS 座標 X+ 方向に 2.0 m 前進、数秒ホバリング
  5) 着陸
- 方針:
  - 使用インタフェース（MAVROS）
    - `/mavros/cmd/arming`（CommandBool）
    - `/mavros/set_mode`（SetMode）→ GUIDED / LOITER 等
    - 位置目標: `/mavros/setpoint_raw/local` または `/mavros/setpoint_position/local`
    - 高度ステップ: `/mavros/cmd/takeoff` またはローカル位置目標
  - シンプルなステートマシン（Python ノード）
    - 遷移: IDLE → ARM → TAKEOFF → HOVER → MOVE_X → HOVER → LAND → DONE
    - パラメータ: takeoff_altitude, dx, hover_seconds
  - 安全:
    - アーム前チェック（IMU, RC, EKF 健全性）
    - タイムアウト時 LOITER 退避／外部停止指示で DISARM

## ステージ2: ミッション拡張
- 複数ウェイポイント（四角・8の字・高度変化）
- 時間ベース vs 収束判定（許容誤差を設ける）
- 速度上限付き（`setpoint_raw/local` で加加速度制限）
- ミッション YAML とローダ（rosparam / yaml）

## ステージ3: 別 PC でのリモート地図生成
- 同一ネットワーク上の別 PC で GLIM を実行:
  - `/livox/lidar`, `/livox/imu` を DDS で購読
  - `/glim_ros/{odom,map,pose}` をネットワークに配信
- 可視化:
  - リモート PC の RViz2 で地図＋軌跡表示
  - rosbag は必要トピックのみ録画（odom, map keyframes, tf）
- 時刻同期:
  - chrony/PTP を推奨
  - `/mavros/timesync_status` 監視

## ステージ4: 障害物検知と飛行停止
- 近接案（短期）
  - PointCloud の最小距離が閾値未満で停止（前方 FOV）
  - `/livox/lidar` を購読するノード → `std_msgs/Bool stop_request` を出力
  - stop_request=true の間はミッションノードが setpoint を抑止
- 中期
  - 前方ボクセル/リングバッファの占有に基づく減速→停止
- 長期
  - サンプリング系のローカルプランナで安全回廊を決定

## ステージ5: 堅牢性と安全
- EKF3 チューニングの範囲と自動フェイルオーバ
  - `/mavros/estimator_status` が不健全 → LOITER 指示
  - 期待時に `/mavros/state` が ARMED でない → ミッション中断
- 無線断対策:
  - RC failsafe → RTL/LOITER
  - ミッションノードのハートビート途絶 N 秒 → setpoint 停止
- バッテリ／熱監視（MAVROS トピック）

## ステージ6: 運用とツール
- スクリプト:
  - `scripts/launch_loiter_tmux.sh`（立ち上げ）
  - `scripts/mission_demo.sh`（追加予定: ステージ1 デモ）
- ドキュメント:
  - `Libox2Glim.md`（ゼロからの復旧手順）
  - `future.md`（ロードマップ／進捗）
- CI（任意）: ブリッジ／ミッションの lint/build チェック

## ステージ7: ミッションノード（実装予定）
- パッケージ: `loiter_mission`
- 機能:
  - パラメトリックなミッション（高度, dx, ホバ秒）
  - 別 PC からの開始／停止サービス
  - ダイアグでフィードバック
- インタフェース:
  - 入力: `/mavros/local_position/pose`, `/mavros/state`, `/mavros/estimator_status`
  - 出力: `/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/setpoint_position/local`

## 備考・参考
- MAVROS の外部推定入口は ODOMETRY-IN（VISION 系はオプション）
- frame は `frame_id=odom` 推奨、`child_frame_id=base_link`
- 分散（covariance）の具体調整は `Libox2Glim.md`（セクション13）を参照
