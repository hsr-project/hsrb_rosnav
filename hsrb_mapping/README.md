
# hsrb_mapping

## パッケージ概要

hsrb_mapping は トヨタパートナーロボット HSR の ROS 地図作成パッケージです．
本パッケージを利用することで環境空間の地図を作成することができます．

ROS Kinetic に対応しています．


## クイックスタート

#### シミュレータの場合

Gazebo シミュレータと RViz を起動します．

```
$ roslaunch hsrb_rosnav_config simple_gazebo_world.launch gui:=true rviz:=true
```

#### 実機ロボットの場合

実機ロボットを使用する場合の ROS 環境に変更して，
重複するノードの停止と RViz の起動を行います．

```
$ hsrb_mode
$ rosnode kill /pose_integrator
$ rviz -d $(rospack find hsrb_rosnav_config)/launch/hsrb.rviz
```


### 地図作成プログラムの実行

下記コマンドを実行して地図作成プログラムを起動します．

- **注意** : 実機ロボットの場合には各ターミナル起動時に `hsrb_mode` を実行して ROS 環境を変更してください．

```
$ roslaunch hsrb_mapping hector.launch
```

![HSR RViz - Begin Mapping](../doc/images/hsrb-rviz_mapping-hector_start.png)

### ロボットの移動操作

RQT Robot Steering やジョイスティックコントローラを用いてロボットの移動操作をします．

#### RQT Robot Steering の利用

rqt を起動します．

```
$ rqt
```

rqt のメニューバーにある Plugins から
Robot Tools > Robot Steering を選択します．
テキストボックス内に移動速度指令のトピック `/hsrb/command_velocity` を設定します．

![HSR RQT - Robot Steering /hsrb/command_velocity](../doc/images/hsrb-rqt_robot-steering_set-topic.png)

スライダを操作してロボットに速度指令を送ります．

#### ジョイスティックコントローラの利用

Xbox 360 互換のジョイスティックコントローラパッドを使用してロボットを動かします．

```
$ roslaunch hsrb_mapping teleop_joy.launch
```

Enable ボタンに設定されている4番のボタン LB (Left Shoulder) ボタンを押しながら
ジョイスティックを操作してロボットに速度指令を送ります．

#### キーボードの利用

キーボード入力から移動速度指令値を出すために次のコマンドを実行します．

```
$ roslaunch hsrb_mapping teleop_keyboard.launch
```

ターミナルに表示されるキーマップに従ってロボットを動かします．


### 地図の作成と保存

ロボットを環境空間内で移動させて地図を広げて行き，
地図が完成したら保存をします．

```
$ rosrun map_server map_saver
```

![HSR RViz - End Mapping](../doc/images/hsrb-rviz_mapping-hector_end.png)

カレントディレクトリに下記の2つのファイルが保存されます．

- map.pgm
- map.yaml

これらのファイルを ROS から指定しやすい場所に移動しておきます．

```
$ mv map.pgm map.yaml ~/.ros/
```

-----

## リファレンス

### /launch

#### hector.launch

- 起動オプション
	- なし
- ノード
  - /hector_mapping
    - 入力トピック
      - /hsrb/base_scan
      - /tf_static
      - /tf
      - /initialpose
      - /syscommand
    - 出力トピック
      - /map
      - /map_metadata
      - /slam_out_pose
      - /slam_cloud
      - /poseupdate
      - /tf
- ソースURL
  - https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/hector.launch


#### gmapping.launch

- 起動オプション
	- なし
- ノード
  - /gmapping
    - 入力トピック
      - /hsrb/base_scan
      - /tf_static
      - /tf
    - 出力トピック
      - /map
      - /map_metadata
      - /gmapping/en
      - /tf
- ソースURL
  - https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/gmapping.launch


#### karto.launch

- 起動オプション
	- なし
- ノード
  - /slam_karto
    - 入力トピック
      - /hsrb/base_scan
      - /tf_static
      - /tf
    - 出力トピック
      - /map
      - /map_metadata
      - /visualization_marker_array
      - /tf
- ソースURL
  - https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/karto.launch


#### teleop_joy.launch

- 起動オプション : デフォルト
	- joy_dev : /dev/input/js0
- ノード
  - /joy_node
    - 入力トピック
      - なし
    - 出力トピック
      - /joy
      - /diagnostics
  - /teleop_twist_joy
    - 入力トピック
      - /joy
    - 出力トピック
      - /hsrb/command_velocity
- ソースURL
	- https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/teleop_joy.launch


#### teleop_keyboard.launch

- 起動オプション
	- なし
- ノード
  - /teleop_twist_keyboard
    - 入力トピック
      - なし
    - 出力トピック
      - /hsrb/command_velocity
- ソースURL
  - https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/teleop_keyboard.launch



### /launch/include

#### play_bag_file.launch.xml

- 起動オプション : デフォルト
	- bag_file : なし
- ノード
  - /robot_state_publisher
    - 入力トピック
      - TBD
    - 出力トピック
      - TBD
  - /player
    - 入力トピック
      - TBD
    - 出力トピック
      - TBD
- ソースURL
	- https://github.com/tork-a/hsrb_ros/blob/master/hsrb_mapping/launch/include/play_bag_file.launch.xml




<!-- EOF -->
