# MIRS MG5 ROS 2 Package with Docker

mirs_mg5の標準的機能を備えたROS 2パッケージ（Docker対応版）

## 主な改変点

このリポジトリは、以下のリポジトリを継承・改変したものです：

1. **[mirs2502](https://github.com/mirs2502)** による改変版
   - オリジナル(mirs240x)をクローンし、パッケージ統合などを行ったもの
2. **[mirs240x](https://github.com/mirs240x)** によるオリジナル版
   - プロジェクトの基盤となるROS 2パッケージ群

### 2026/01 リファクタリング
- **Dockerfileの整理**: `apt-get` による手動インストールを最小限にし、`rosdep` による自動依存関係解決に移行しました。
- **ビルド効率の向上**: コンテナ内にビルド用エイリアスを追加し、開発効率を改善しました。
- **デバイス権限の適正化**: `group_add` によるアクセス権限設定を導入しました。

### Dockerを使用するメリット
- **環境構築が簡単**: ROS 2 Humbleと、lidarやmicro-ros-agentといったros2パッケージのインストール不要
- **依存関係が自動解決**: `mirs/package.xml` に基づき `rosdep` が自動で必要なパッケージをインストール
- **再現性が高い**: どのマシンでも同じ環境で実行可能
- **クリーンな環境**: ホストシステムを汚さない

## 要件

### ハードウェア

- cugo v3i
- ESP32
- PC
- Cytron MD10C

### ソフトウェア

- Docker (Docker Desktop for Windowsは使用不可)
- Docker Compose
- X11サーバー (Windowsの場合WSLg)
- USBIPD-WIN [GitHub](https://github.com/dorssel/usbipd-win) (WindowsのみUSB接続のために必要)
- Arduino IDE (ESP32にmicro-ros-clientを導入するため)

## 含まれるパッケージ

### mirs
メインパッケージ。以下の機能を提供：
- ESP32との通信（micro-ROS）
- オドメトリ計算とTF配信
- ロボットモデル（URDF）
- navigation2 / SLAM
- ジョイスティックによるロボット制御

### mirs_msgs
カスタムメッセージ定義パッケージ

## つかいかた

### 1. ESP32のセットアップ

Arduino IDEに以下のソースコードとライブラリを導入する
- [mirs2502/mirs2502_esp32(esp32用ソースコード)](https://github.com/mirs2502/mirs2502_esp32.git)
    - 2502の元になったコード[mirs240x/mirs24_esp32(esp32用ソースコード)](https://github.com/mirs240x/mirs24_esp32.git)
  - [mirs240x/micro_ros_arduino_mirs240x](https://github.com/mirs240x/micro_ros_arduino_mirs240x)
    - micro-rosライブラリ。zipでインポート

    ボードの選択についてはArduinoIDEにespのボードマネージャのバージョン2.x系を導入する必要があり、必要に応じてArduinoIDEをダウングレードしてください。

    また、ボードについてはesp32 dev moduleを実行環境では使用しました。

### 2. リポジトリのクローン

```bash
git clone https://github.com/6kam/MirsTanq_A.git
cd MirsTanq_A/mirsws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git

# ここで、srcディレクトリのしたにmicro-ROSやslllidar_ros2が配置されているか確認する
```

### 3. Docker イメージのビルド

```bash
cd ..
docker compose build
```

### 4. コンテナの起動

```bash
xhost +local:docker  # X11転送を許可（初回のみ）
docker compose up -d
docker compose exec mirs bash
```

### 5. ROS 2 ノードのビルドと実行

#### ビルド
コンテナ内で便利なエイリアスが使用可能です
- `ru`: `rosdep update`
- `ri`: `rosdep install --from-path src --ignore-src -r -y`
- `cb`: `colcon build --symlink-install` (全ビルド)
- `cbs`: `colcon build --symlink-install --packages-select` (パッケージ指定)
- `cbt`: `colcon build --symlink-install --packages-up-to` (依存込み)
- `si`: `source install/setup.bash`

コンテナ内では次のコマンドは起動時に宣言されています

- `source /opt/ros/humble/setup.bash`

コンテナ内で以下のコマンドを実行してください

```bash
ru
ri
cb
si
```

#### 実行
実行前にLiDARとESP32をPCにUSB接続してください。

`docker-compose.yml` で以下のデバイスがマウントされています（接続時）：
- `/dev/ttyUSB0`: LiDAR
- `/dev/ttyUSB1`: ESP32

先にlidarを接続することでttyUSB0がLiDARになるはずです。launchのときにusbポートを指定する必要はありません。
```bash
# 基本的なシステム起動
ros2 launch mirs mirs.launch.py

# ロボットモデルの可視化（RViz2）
ros2 launch mirs display.launch.py

# SLAM実行
ros2 launch mirs pc_slam.launch.py

# ナビゲーション
ros2 launch mirs navigation.launch.py
```

## 開発方法

### ソースコードの編集

ホストマシンの `src/` はコンテナ内でマウントされているので、コンテナ内で編集してもホストに反映されます。
コンテナ内で編集したときは、`cb` でビルドできます。

### デバッグ

```bash
# ノードのリスト表示
ros2 node list

# トピックのリスト表示
ros2 topic list

# トピックのデータ確認
ros2 topic echo /odom
ros2 topic echo /encoder #cugov3はクローラのため値は2つ出ます

# TFツリーの確認
ros2 run tf2_tools view_frames
```

## コンテナの終了方法
```bash
exit
docker compose down
```

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は [LICENSE](LICENSE) を参照してください。

## 謝辞

このプロジェクトは、以下の先行開発の成果を継承しています：

- **mirs2502** ([GitHub](https://github.com/mirs2502))
- **mirs240x** ([GitHub](https://github.com/mirs240x))

開発に携わった皆様に感謝申し上げます。

## 参考リンク

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [micro-ROS](https://micro.ros.org/)
