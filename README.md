# MIRS MG5 ROS 2 Package with Docker

mirs_mg5の標準的機能を備えたROS 2パッケージ（Docker対応版）

## 主な改変点

このリポジトリは、以下のリポジトリを継承・改変したものです：

1. **[mirs2502](https://github.com/mirs2502)** による改変版
   - オリジナル(mirs240x)をクローンし、パッケージ統合などを行ったもの
2. **[mirs240x](https://github.com/mirs240x)** によるオリジナル版
   - プロジェクトの基盤となるROS 2パッケージ群
     
### 問題点
- **TFツリー**: base_footprintとbase_linkについて動作の条件が把握できていない。現状はbase_linkを採用。
- **joystickについて**: 正常な動作が確認できていません。暴走するのでlaunchしないことを推奨。
- **nav2 params**: コストマップについての調整がまだ詰め切れていません。cloneのままの環境での動作ができなければそこを調整してください。
- **map**: pc_slam.launch.pyでmapを作成したものをそのまま使用すると正常に動作しない可能性がある。計測についてのパラメータを調整するか、ペイントアプリなどで点のまばらなところの塗りつぶしや、足跡が壁として認識された場合には白く塗りつぶすなどする。

## 要件

### ハードウェア

- cugo v3i
- ESP32
- PC
- Cytron MD10C

### ソフトウェア

- Linux (Arch Linux(Hyprland), Ubuntu 24.04, WSL2上のArch Linuxで動作確認済み)
- Docker (Docker Desktopは不可)
- Docker Compose
- X11サーバー (Windowsの場合WSLg)
- USBIPD-WIN [GitHub](https://github.com/dorssel/usbipd-win) (Windows WSL2環境のみUSB接続のために必要)
- Arduino IDE (ESP32にmicro-ros-clientを導入するため)

## 含まれるパッケージ

### mirs

- ESP32との通信（micro-ROS）
- オドメトリ計算とTF配信
- ロボットモデル（URDF）
- navigation2 / SLAM

### mirs_msgs

カスタムメッセージ定義パッケージ

## つかいかた

### 1. ESP32のセットアップ

Arduino IDEに以下のソースコードとライブラリを導入する
- esp32用ソースコード[mirs-tanq-a-esp](https://github.com/6kam/mirs_tanq_a_esp)
    - 上のソースコードの元になったコード[mirs240x/mirs24_esp32(esp32用ソースコード)](https://github.com/mirs240x/mirs24_esp32.git)
- micro-rosライブラリ。zipでインポート [mirs240x/micro_ros_arduino_mirs240x](https://github.com/mirs240x/micro_ros_arduino_mirs240x)

    ボードの選択についてはArduinoIDEにespのボードマネージャのバージョン2.x系を導入する必要があります。
    また、ボードについてはesp32 dev moduleを使用してください。

### 2. リポジトリのクローン

```bash
git clone https://github.com/6kam/MirsTanq_A.git
cd mirs-tanq-a/src
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
- `mirs`: `ros2 launch mirs mirs.launch.py`
- `slam`: `ros2 launch mirs slam.launch.py`
- `nav`: `ros2 launch mris nav.launch.py`

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
mirs

# SLAM実行
slam

# ナビゲーション
nav
```

## 開発方法

### ソースコードの編集

ホストマシンの `src/` はコンテナ内でマウントされているので、ホストで編集した内容が起動中のコンテナにすぐに反映されます。

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

このプロジェクトは、以下の先行開発の成果を継承しています

- **mirs2502** ([GitHub](https://github.com/mirs2502))
- **mirs240x** ([GitHub](https://github.com/mirs240x))

開発に携わった皆様に感謝申し上げます。

## 参考リンク

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [micro-ROS](https://micro.ros.org/)
