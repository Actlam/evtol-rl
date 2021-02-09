# evtol-rl
eVTOLによるwaypointを使ったQ-Learningの実装<br>
pwmの累計により消費電力を推定し報酬値から減算することでhoverモードとplaneモードの使い分けを消費電力に基づき自動化する<br>
シミュレーション環境はDronecodeのドキュメントで紹介されているGazebo Simulationを使用

## Enviroment
- Ubuntu 18.04 LTS
- ROS Melodic

## Installation
- [PX4, Gazeboのインストール](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#gazebo-jmavsim-and-nuttx-pixhawk-targets)
- [ROSのセットアップ](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo)
- [MAVROSのインストール](https://docs.px4.io/master/en/ros/mavros_installation.html)

## how to use
- 環境のインストールを完了
- `catkin_ws/src` 配下にenergyを配置
- インストールした`PX4-Autopilot`リポジトリのmavrosのlaunchファイルで使用する機体をstandardVTOLに指定する
  - `PX4-Autopilot/launch/mavros_posix_sitl.launch`のvehicle設定を書き換える
   ```diff
   - <arg name="vehicle" default="iris"/>
   + <arg name="vehicle" default="standard_vtol"/>
   ```

### 学習の実行
#### Gazebo環境の起動
```shell
# cloneしてきたpx4のリポジトリに入る
$ cd src/PX4-Autopilot_default

# pathを通す
$ DONT_RUN=1 make px4_sitl_default gazebo
$ source ~/catkin_ws/devel/setup.bash    # (optional)
$ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# ROS上でpx4とgazeboとmavrosが立ち上がるlaunchファイルを呼び出す
roslaunch px4 mavros_posix_sitl.launch
```

#### 学習用ノードの起動
別のTerminalで学習用ノードを立ち上げる
```shell
$ cd catkin_ws
$ rosrun energy qlearn.py
```

## 学習データの分析
追加予定
