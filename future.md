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

## ステージ4.5: ゆっくり飛びながらの地図生成と安全経路の自動作成
- 目的: 低速で移動しつつ、オンラインで占有/ESDF 地図を更新し、安全な短ホライズン経路を自動生成→追従。
- 入力: `/glim_ros/odom`, `/livox/lidar`
- 地図: ダウンサンプル→地面分離→占有/ESDF（OctoMap/Voxblox/NVBlox 等）
- ローカルプランナ（2–5 Hz 再計画）:
  - 2D: DWA/TEB + 膨張コストマップ（機体半径＋マージン）
  - 3D: RRT* or ESDF 勾配ベース（狭所は減速し閾値チェック）
- 追従: `/mavros/setpoint_raw/local` で 20 Hz 配信（vx,vy≤0.5 m/s, vz≤0.3 m/s, yaw_rate≤15°/s）
- 安全: 近接停止、地図未更新時は減速→停止、RC 介入即 LOITER

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

## ステージ6.5: 充電ステーションへの自動着陸（ドッキング）
- 誘導方式（いずれか）:
  - マーカー: 下向きカメラ + AprilTag/ArUco でステーション姿勢推定 → XY/yaw 微修正 → 最終降下
  - Precision Landing（ArduPilot）: PLND（外部ビジョン）を `/mavros/landing_target/pose_in` で供給（PLND_ENABLED=1, PLND_TYPE=1）
  - ビーコン: UWB/IR/LiDAR ビーコンで XY 補正
- ステーション設計:
  - 接点式（整列コーン/ファンネル） or 非接触給電（Qi/独自）
  - 接触検出・電圧監視・極性/短絡保護・耐候
  - マーカーは中央＋補助タグでロバスト化
- 手順（例）:
  1) GLIM 座標で上空（≈2 m）へ移動
  2) 降下しながらタグ捕捉（≈0.5–1.0 m）→ XY/yaw 微修正（ビジュアルサーボ）
  3) LAND_SPEED 低速で最終降下 → 接触検出 → モータ停止 → 充電開始
  4) 充電完了後、再アーム可能化
- 設定の要点:
  - MAVROS/ArduPilot: PLND パラメータ（PLND_ENABLED, PLND_TYPE）と LAND_SPEED、RNGFND（近接高度）
  - リスク対策: 照明/反射、風・地面効果、喪失時リトライ/退避（LOITER）

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

---

## ステージ8: 手動飛行の「記録・再生（逆再生含む）」

### 8.1 目的
- 手動でうまく飛べた経路を、そのまま（または逆順に）自動再生。

### 8.2 記録（Recorder）
- 記録対象（どちらか）:
  - `/mavros/local_position/pose`（EKF 出力）
  - `/glim_ros/odom`（GLIM 出力）
- 推奨: 「開始点原点化の相対軌跡」で保存（再生時に現在位置へ平行移動しやすい）。
- 実装案:
  - rosbag: `ros2 bag record -o flight1 /mavros/local_position/pose`
  - もしくはノードで CSV/YAML に `timestamp,x,y,z,yaw` を保存（開始/停止サービス）

### 8.3 再生（Replayer）
- 出力先（いずれか）:
  - 簡易: `/mavros/setpoint_position/local`（20Hz 程度）
  - 高度: `/mavros/setpoint_raw/local`（速度・加速度・yaw を明示制御）
- 処理:
  1) 記録した相対軌跡を「現在の開始位置」に平行移動
  2) スプライン補間＋速度/加速度上限で滑らかに
  3) 時系列で setpoint を配信（到達判定→次点）

### 8.4 逆再生（バック）
- 方法: 点列を「終端→始端」の順にたどって setpoint を配信。
- yaw の扱い（いずれか）:
  - 軌跡接線方向へ向ける（自然）
  - 機体は前方のまま後退（yaw 固定 / yaw_rate=0）

### 8.5 安定化のコツ
- フレーム: `odom` 推奨（相対復元向き）。
- 速度・加速度上限を厳しめに（急激な戻りを防止）。
- 許容誤差で到達判定（距離・yaw）＋タイムアウト時 LOITER 退避。
- 風でのズレは次点へ遷移しながら補正。

### 8.6 安全
- RC 介入で即 LOITER。
- ジオフェンス/高度上限/障害物停止信号（stop_request）で setpoint 抑止。

### 8.7 実装予定（loiter_mission に追加）
- `loiter_mission_recorder`: CSV/YAML へ記録（開始/停止サービス）。
- `loiter_mission_replayer`: CSV/YAML から再生（逆再生/速度上限/到達判定/補間パラメータ）。
