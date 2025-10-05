# MAVROS インストール・起動手順

## 概要
ROS 2 Humble環境でのMAVROS（MAVLink ROS）のインストールとArduPilot用の起動手順です。
参考: [MID360 reinstall.md](https://github.com/nonsaya/MID360/blob/main/reinstall.md)

## 環境
- OS: Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS: ROS 2 Humble
- 対象デバイス: Jetson Orin Nano
- シリアルポート: `/dev/ttyTHS1` (921600bps)

## 1. Ubuntu標準リポジトリの有効化

Jetson環境では標準リポジトリがコメントアウトされているため、まず有効化が必要です。

```bash
# バックアップを作成
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# Jetson向けの正しいリポジトリに置き換え
sudo tee /etc/apt/sources.list > /dev/null << 'EOF'
# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://ports.ubuntu.com/ubuntu-ports/ jammy main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ jammy-security main restricted universe multiverse
EOF
```

## 2. パッケージリストの更新

```bash
sudo apt-get update
```

## 3. MAVROS依存パッケージのインストール

```bash
# GeographicLib関連パッケージをインストール
sudo apt-get install -y geographiclib-tools libgeographic-dev libgeographic19
```

## 4. MAVROS本体のインストール

```bash
# MAVROS本体とExtrasをインストール
sudo apt-get install -y ros-humble-mavros ros-humble-mavros-extras
```

インストールされるパッケージ：
- `ros-humble-mavros` (2.12.0-1)
- `ros-humble-mavros-extras` (2.12.0-1)
- `ros-humble-libmavconn` (2.12.0-1)
- `ros-humble-mavlink` (2025.9.9-1)
- その他依存パッケージ

## 5. GeographicLibデータセットの導入

```bash
# GPS/地理系機能に必要なデータセットをインストール
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

インストールされるデータセット：
- **egm96-5**: 地球重力場モデル
- **egm96**: 重力データ
- **emm2015**: 地磁気モデル

## 6. シリアル権限の設定

```bash
# 現在のユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER

# グループ設定を即座に反映（再ログイン不要）
newgrp dialout

# 権限確認
ls -l /dev/ttyTHS1
# 出力例: crw-rw---- 1 root dialout 240, 1 11月 22  2023 /dev/ttyTHS1
```

## 7. MAVROS起動

### ArduPilot用起動コマンド

```bash
# ROS2環境を読み込み
source /opt/ros/humble/setup.bash

# MAVROSをArduPilot用に起動（/dev/ttyTHS1, 921600bps）
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600
```

### 動作確認

別ターミナルで以下を実行：

```bash
# ROS2環境を読み込み
source /opt/ros/humble/setup.bash

# トピック一覧確認
ros2 topic list | grep mavros

# 接続状態確認
ros2 topic echo /mavros/state

# IMUデータレート確認
ros2 topic hz /mavros/imu/data
# 出力例: average rate: 30.301
```

## 8. トラブルシューティング

### 依存関係エラー
- Ubuntu標準リポジトリが有効化されていない場合に発生
- `sources.list`の設定を確認

### 権限エラー
- `dialout`グループに所属していない場合に発生
- 一度ログアウト/ログインして`dialout`グループを確実に反映

### 接続できない
- ArduPilotが`/dev/ttyTHS1`で起動しているか確認
- シリアルポートの設定（ボーレート921600）を確認

## 利用可能なlaunchファイル

```bash
# 確認方法
ls -la /opt/ros/humble/share/mavros/launch/

# 利用可能なlaunchファイル
# - apm.launch     : ArduPilot用（XML形式）
# - px4.launch     : PX4用
# - node.launch    : 汎用ノード起動
# - multi_uas.launch: 複数UAS用
```

## 参考情報

- [MAVROS公式ドキュメント](http://wiki.ros.org/mavros)
- [ArduPilot公式サイト](https://ardupilot.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
