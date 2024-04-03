
# hsrb_mapping

## パッケージ概要

hsrb_mapping は トヨタパートナーロボット HSR の ROS 地図作成パッケージです．
本パッケージを利用することで環境空間の地図を作成することができます．

ROS Foxy に対応しています．


## クイックスタート

#### シミュレータの場合

整備中


#### 実機ロボットの場合

実機ロボット立ち上げと RViz の起動を行います．

```
$ colcon_cd hsrb_mapping && rviz2 -d rviz/hsr_slam_toolbox_mapping.rviz
```


### 地図作成プログラムの実行

下記コマンドを実行して地図作成プログラムを起動します．


```
$ ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```

### ロボットの移動操作

RQT Robot Steering やジョイスティックコントローラを用いてロボットの移動操作をします．

#### RQT Robot Steering の利用
事前にRQTのプラグインをインストールしておきます．

```
$ apt install ros-foxy-rqt-robot-steering
```

rqt を起動します．

```
$ rqt
```

rqt のメニューバーにある Plugins からRobot Tools > Robot Steering を選択します．
※ Robot SteeringがRobot Tools内に無い場合は，以下オプションを追加してrqt起動を試してみてください．
```
$ rqt --force-discover
```

テキストボックス内に移動速度指令のトピック `/omni_base_controller/command_velocity` を設定します．
スライダを操作してロボットに速度指令を送ります．


#### ジョイスティックコントローラの利用

DUALSHOCK3/4 互換のジョイスティックコントローラパッドを使用してロボットを動かします．
```
$ ros2 launch hsrb_mapping teleop.launch.py
```

※ 事前に以下パッケージをインストールする必要がある場合があります．
```
$ apt install ros-foxy-joy-linux ros-foxy-joy-linux-dbgsym
```

Enable ボタンに設定されている10番のボタン L1 ボタンを押しながら
ジョイスティックを操作してロボットに速度指令を送ります．

#### キーボードの利用

キーボード入力から移動速度指令値を出すために次のコマンドを実行します．

```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/omni_base_controller/command_velocity
```

ターミナルに表示されるキーマップに従ってロボットを動かします．


### 地図の作成と保存

ロボットを環境空間内で移動させて地図を広げて行き，
地図が完成したら保存をします．

```
$ ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000
```

カレントディレクトリに下記の2つのファイルが保存されます．

- map.pgm
- map.yaml

これらのファイルを ROS から指定しやすい場所に移動しておきます．

```
$ mv map.pgm map.yaml ~/.ros/
```

<!-- EOF -->
