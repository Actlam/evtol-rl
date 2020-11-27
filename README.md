# evtol-rl
eVTOLによるwaypointを使った強化学習
シミュレーション環境はDronecodeのドキュメントで紹介されているGazebo Simulationを使用

## Enviroment
- Ubuntu 18.04 LTS
- ROS Melodic

## Installation
- [px4のビルド環境]()
- [ROSのセットアップ]
- [Launching Gazebo with ROS Wrappers](https://dev.px4.io/master/en/simulation/ros_interface.html)
- [MAVROSのインストール]

## how to use
### setup
- 環境のインストールを完了
- `catkin_ws/src` 配下にenergyを配置
- インストールした`PX4-Autopilot`リポジトリのmavrosのlaunchファイルで使用する機体をstandardVTOLに指定する
  - `PX4-Autopilot/launch/mavros_posix_sitl.launch`のvehicle設定を書き換える
   ```
   <arg name="vehicle" default="iris"/>
   ```
   を以下のように修正
   ```
   <arg name="vehicle" default="standard_vtol"/>
   ```

### 学習の実行
#### Gazebo環境の起動
```
# cloneしてきたpx4のリポジトリに入る
$ cd src/PX4-Autopilot_default

# pathを通す
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# ROS上でpx4とgazeboとmavrosが立ち上がるlaunchファイルを呼び出す
roslaunch px4 mavros_posix_sitl.launch
```

#### 学習用ノードの起動
別のTerminalで学習用ノードを立ち上げる
```
cd catkin_ws
rosrun energy qlearn.py
```