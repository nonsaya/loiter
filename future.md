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

### ステージ1 検討詳細（MVP/TDD）
- ゴール/完了条件（受け入れ基準）
  - LOITER 安定から開始し、無線/サービスによる開始合図で以下が自動完了:
    - 離陸: 目標高度（例 2.0 m）に ±0.3 m 以内で到達（≤5 s）、速度過大でない
    - ホバ: 指定秒数（例 3 s）で高度変動 ±0.2 m 以内
    - 前進: X+ に指定距離（例 2.0 m）±0.3 m、偏差収束（≤6 s）
    - 着陸: LAND で接地検出→自動モータ停止（安全な降下率）
  - いつでも RC 介入で即 LOITER に遷移（スティック/モード切替で中断）
  - いずれかタイムアウト/不健全時は LOITER へ退避、setpoint 配信停止

- 運用フロー（案）
  1) 事前: GLIM+EKF3 正常、`/mavros/state` 接続、timesync 正常
  2) ミッション開始: LOITER→GUIDED へ切替、arming 実施
  3) 離陸: takeoff または Z 目標 setpoint（20 Hz 配信）
  4) ホバ: 収束確認（速度/位置）→規定秒待機
  5) 前進: X+ の相対 setpoint（速度上限を保守的に）
  6) 着陸: LAND モード（または Z 下降目標）→接地 → DISARM/終了

- インタフェース（最小）
  - 入力: `/mavros/state`, `/mavros/estimator_status`, `/mavros/local_position/pose`
  - 出力: `/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/setpoint_position/local`（MVP）
  - 周期: setpoint は 20 Hz（ArduPilot は連続 setpoint を要求）

- ステートマシン詳細
  - IDLE → ARM（前提: EKF 健全, RC OK, timesync OK）
  - ARM → TAKEOFF（GUIDED へモード変更成功）
  - TAKEOFF → HOVER（高度誤差と速度で収束判定 or タイムアウト）
  - HOVER → MOVE_X（規定秒待機後、相対 X 目標に切替）
  - MOVE_X → HOVER（位置収束 or タイムアウト）
  - HOVER → LAND（LAND へモード変更）→ DONE
  - いずれの状態でもフェイルセーフ条件で LOITER へフォールバック

- 安全/フェイルセーフ
  - 事前: RC failsafe 設定, geofence/高度上限, EKF 正常性確認
  - 実行中: 速度/姿勢が閾値超過→減速→LOITER、トピック途絶→LOITER
  - 外部停止: サービス/RC で中断→LOITER、DISARM は地上/低高度でのみ

- パラメータ（例）
  - `takeoff_altitude=2.0`, `dx=2.0`, `hover_seconds=3`
  - 速度上限: `vx,vy≤0.5 m/s, vz≤0.3 m/s, yaw_rate≤15°/s`（MVP）

- 設定確認（ArduPilot/MAVROS）
  - GUIDED 受入可（GUIDED/NAV_GUIDED 有効）
  - LAND の降下率/接地検出、EKF3 ソース（外部オドメトリ）
  - timesync 正常（`/mavros/timesync_status`）

- TDD/MVP 進め方（実装は本ブランチでは未実施）
  - ユニット: ステート遷移（正常/タイムアウト/中断）をモック時刻で検証
  - インテグレーション: トピック入出力のレート/内容をロガで検証
  - SITL ベンチ: ArduPilot SITL + MAVROS でシナリオ再現（屋外前）
  - 受入試験: 低高度・広場で段階試験（離陸→ホバ→着陸→前進追加）

- 実装計画（将来の別ブランチ）
  - ROS 2 パッケージ: `loiter_mission`（`mission_node` のみ, MVP）
  - 起動/停止サービスとダイアグトピック（進捗/状態/理由）
  - デモ: `scripts/mission_demo.sh`（開始/中断, ログ収集）

- リスク/留意点
  - 風/地面効果での過制御、GLIM/通信遅延、setpoint 途絶
  - RC の優先制御を常に確保、緊急時の LOITER/RTL 動作確認

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

---

## ステージ8.5: 機械学習を用いた高度ミッション

### 8.5.1 模倣学習による航路追従
- 目的: 手動飛行のデータセット（pose/yaw/速度）から挙動を学習し、同等の滑らかな追従を自律で再現。
- 入出力: 過去の局所状態（相対座標・障害物特徴）→ 次時刻の速度/姿勢コマンド。
- データ: 既存の「記録・再生」の記録を学習用に流用。
- 評価: 追従誤差、介入回数、停止率。

### 8.5.2 物体認識ベースのミッション化
- 目的: カメラでタグ/対象物を検出→ GLIM 座標へ投影→ 自動ウェイポイント生成。
- 用途: ドッキング、対象物まで接近/撮影/離脱など。

### 8.5.3 学習型ローカルプランナ（古典補完）
- 目的: DWA/TEB/RRT* のクラシックに学習コストを加え、安全/快適/滑らかさを統計的に補正。
- 成果: クリアランス違反率↓、曲率変化↓、到達率↑。

### 8.5.4 異常検知・フェイルセーフ
- 目的: IMU/電流/姿勢/風推定から「異常兆候」を学習（One-Class/AE/IsolationForest）。
- 動作: 兆候検知で LOITER/RTL へ早期移行。

### 8.5.5 航続・バッテリ予測
- 目的: 速度・風・重量などから残航続/帰還可否を回帰学習→ ミッション分岐（早期帰還/着陸）。

### 8.5.6 セマンティクス地図による可否判断
- 目的: LiDAR + カメラで地面分離/セマンティクス（歩道/芝/水面）を学習、通行可否を強化。

### 8.5.7 学習型ランディング補助
- 目的: 下向きカメラの特徴点トラッキング＋小型ネットで XY 微修正ゲインを学習（最終整列の頑健化）。

### 優先導入順
1) 模倣学習の航路追従（記録・再生の延長で導入容易）
2) 物体認識ベースのミッション化（タグ接近/ドッキング）
3) 異常検知・バッテリ予測（安全性向上）
4) 学習型ローカルプランナ（段階導入、古典にフォールバック）

## ステージ9: ミッションノード（実装予定）
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

## ステージ10: 手動飛行の「記録・再生（逆再生含む）」

### 10.1 目的
- 手動でうまく飛べた経路を、そのまま（または逆順に）自動再生。

### 10.2 記録（Recorder）
- 記録対象（どちらか）:
  - `/mavros/local_position/pose`（EKF 出力）
  - `/glim_ros/odom`（GLIM 出力）
- 推奨: 「開始点原点化の相対軌跡」で保存（再生時に現在位置へ平行移動しやすい）。
- 実装案:
  - rosbag: `ros2 bag record -o flight1 /mavros/local_position/pose`
  - もしくはノードで CSV/YAML に `timestamp,x,y,z,yaw` を保存（開始/停止サービス）

### 10.3 再生（Replayer）
- 出力先（いずれか）:
  - 簡易: `/mavros/setpoint_position/local`（20Hz 程度）
  - 高度: `/mavros/setpoint_raw/local`（速度・加速度・yaw を明示制御）
- 処理:
  1) 記録した相対軌跡を「現在の開始位置」に平行移動
  2) スプライン補間＋速度/加速度上限で滑らかに
  3) 時系列で setpoint を配信（到達判定→次点）

### 10.4 逆再生（バック）
- 方法: 点列を「終端→始端」の順にたどって setpoint を配信。
- yaw の扱い（いずれか）:
  - 軌跡接線方向へ向ける（自然）
  - 機体は前方のまま後退（yaw 固定 / yaw_rate=0）

### 10.5 安定化のコツ
- フレーム: `odom` 推奨（相対復元向き）。
- 速度・加速度上限を厳しめに（急激な戻りを防止）。
- 許容誤差で到達判定（距離・yaw）＋タイムアウト時 LOITER 退避。
- 風でのズレは次点へ遷移しながら補正。

### 10.6 安全
- RC 介入で即 LOITER。
- ジオフェンス/高度上限/障害物停止信号（stop_request）で setpoint 抑止。

### 10.7 実装予定（loiter_mission に追加）
- `loiter_mission_recorder`: CSV/YAML へ記録（開始/停止サービス）。
- `loiter_mission_replayer`: CSV/YAML から再生（逆再生/速度上限/到達判定/補間パラメータ）。

---

## 専用ビューワ計画（Ubuntu → Windows）
- MVP（Ubuntu, Python）: Open3D + ROS 2
  - `/livox/lidar` 購読 → PointCloud2 変換 → Open3D 表示
  - 色付け（強度/時間/リング）、ボクセルDS、録画/再生、FPS/遅延表示
- 高機能（C++）: Qt + VTK/PCLVisualizer（または RViz2 プラグイン）
  - 計測（距離/面/体積）、ROI/クリッピング、スクショ/動画
  - LOD/タイル/GPU点群で大規模最適化
- Windows移植:
  - ROS 2 Windows か、独自通信（TCP/UDP/gRPC）＋スタンドアロンViewer

---

## アイデア集（実現性 低 → 高）
- マルチ機・挑戦
  - 領域分割カバレッジ、リレー通信、移動体着陸、事前地図からの自動点検経路
- 運用自動化
  - データ収集→クラウドKPI、夜間巡回（ドッキング運用）
- HMI/可視化
  - 専用ビューワ、RViz2 HUD、AR アノテーション
- ML/AI
  - 模倣学習、学習型ローカルプランナ、異常検知、オンボード最適化
- アクティブセンシング
  - 視点最適化、ロバスト再ローカライズ
- ビジョン
  - 物体検出タスク、マーカー無し精密着陸、サーマル/マルチスペクトル
- マップ・経路
  - フロンティア探索、ESDF の k-最短、地形追従、セマンティクス地図
- 実運用強化
  - 風推定＋対風制御、バッテリ学習予測→RTL/ドッキング分岐
  - フェイルセーフ拡張、時刻同期の健全性監視
