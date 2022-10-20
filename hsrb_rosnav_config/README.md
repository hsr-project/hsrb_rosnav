
# hsrb_rosnav_config

## パッケージ概要

hsrb_rosnav_config は トヨタパートナーロボット HSR の ROS 自律移動パッケージです．
本パッケージを利用することで環境空間内の自律移動を行うことができます．

ROS Foxy に対応しています．


## チュートリアル

URL : TBD


## クイックスタート

### ソフトウェアの起動

#### シミュレータの場合

整備中

#### 実機ロボットの場合

実機ロボット立ち上げと RViz の起動を行います．
```
$ colcon_cd hsrb_rosnav_config && rviz2 -d rviz/hsr_navigation2.rviz
```


### 自律移動プログラムの実行

下記コマンドを実行して自律移動プログラムを起動します．
引数mapには事前に作成した地図のyamlファイルへのフルパスを入力します．

```
$ ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/map.yaml
```

### ロボットの初期位置の設定

ロボットは起動時に地図のどこに居るのかを知りません．
RViz を使って地図上でのロボットの初期位置と初期姿勢を与えます．

1. RViz 上部にある **2D Pose Estimate** をクリック
2. RViz のロボット空間内で目標位置でクリックしてそのままドラッグして方向を指定

RViz 上のロボットの位置と姿勢が指定した地点に設定されます．

### RViz での自律移動操作

RViz でロボットが自律移動する目標位置と方向を指定します．

1. RViz 上部にある **Navigation2 Goal** をクリック
2. RViz のロボット空間内で目標位置でクリックしてそのままドラッグして方向を指定

目標の位置・方向に到達して自律移動が終了します．

<!-- EOF -->
