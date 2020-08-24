
# はじめに

## 本チュートリアルについて

本チュートリアルは
トヨタパートナーロボット HSR の ROS パッケージ hsrb_rosnav 内の

- 地図作成パッケージ : hsrb_mapping
- 自律移動パッケージ : hsrb_rosnav_config

の使用方法について説明します．
本チュートリアルの構成は次のようになっています．

- [はじめに（本章）](hsrb_ros-tutorial_ja_introduction.md)
- [地図の作成](hsrb_ros-tutorial_ja_mapping.md)
- [自律移動](hsrb_ros-tutorial_ja_navigation.md)
- [トラブルシューティング](hsrb_ros-tutorial_ja_trouble-shooting.md)


## ソフトウェアのインストール

### システム要件

本チュートリアルで使用するシステムの要件を下に記します．

- Ubuntu 16.04 64bit
- ROS Kinetic


### ROS と HSR ソフトウェアのインストール

ROS および HSR ソフトウェアのインストールを下記URLにある
「**HSRマニュアル**」の「**開発PCのセットアップ**」に従って行ってください．

既にこれらのソフトウェアがインストールされている場合は
次の「**hsrb_ros パッケージのインストール**」に進んでください．

- ROS Kinetic の Ubuntu へのインストール
 - [http://wiki.ros.org/ja/kinetic/Installation/Ubuntu][ab085263]

  [ab085263]: http://wiki.ros.org/ja/kinetic/Installation/Ubuntu "ROS Kinetic Installation on Ubuntu"

- HSR マニュアル
  - Docs » 6. HSRの使い方 » 6.2. HSRを使うためのセットアップ » 6.2.1. 開発PCのセットアップ
  - [https://docs.hsr.io/manual/howto/pc_install.html][29d6e905]

  [29d6e905]: https://docs.hsr.io/manual/howto/pc_install.html "HSR - PC Install"


### hsrb_ros パッケージのインストール

Toyota HSR2015（hsrb）用の ROS の地図作成・自律移動パッケージをインストールします．

まず rosinstall，wstools ををインストールします．

```
$ sudo apt-get update
$ sudo apt-get install python-wstool
```

本チュートリアル用に以下のように新しくワークスペースを作成してください．
ここでは `catkin_ws` という名前にしています．

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ wstool init src
$ wstool set -t src --git hsrb_ros https://github.com/tork-a/hsrb_ros.git
$ wstool update -t src
$ rosdep update && rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

コマンドは一部長いものがあるので表示環境により改行される場合がありますが，
実際には `$` より後の連続した一つのコマンドとして入力してください．

- wstool : GitHub などにあるソースコードを自動的に取得しワークスペースに
展開するツール
  - [http://wiki.ros.org/wstool](http://wiki.ros.org/wstool)
- rosdep : ソースコードが依存するシステムパッケージを自動でインストールするためのコマン
ド
  - [http://wiki.ros.org/ja/ROS/Tutorials/rosdep](http://wiki.ros.org/ja/ROS/Tutorials/rosdep)


以上でソフトウェアの基本的なインストール作業は終了です．

ROS やワークスペースが正しくインストール，設定されているか確認したい場合は
「**トラブルシューティング**」の「**ROS 環境の確認**」を参考にしてください．

- 【参考】[ROS 環境の確認](hsrb_ros-tutorial_ja_trouble-shooting.md#checking-ros-environment)
  - [hsrb_ros-tutorial_ja_trouble-shooting.md#checking-ros-environment](hsrb_ros-tutorial_ja_trouble-shooting.md#checking-ros-environment)


#### ターミナル起動時に自動的に setup.bash を実行する設定

`source devel/setup.bash` はターミナルを開くたびに実行する必要があります．
毎回このコマンドを実行するのは面倒なので
ターミナル起動時に自動的に `source` コマンドが実行されるようにします．

次のコマンドを実行して `source devel/setup.bash` コマンドを
ファイル `~/.bashrc` に追加記入します．

- **注意** : 下記コマンドの `>>` を `>` とすると `~/.bashrc` が上書きされて設定が消えてしまうので気をつけてください．

```
$ echo "### For HSR ROS Navigation" >> ~/.bashrc
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

ROS のワークスペースは各自の作業に応じて切り替えるのが一般的な使い方です．
本チュートリアル終了後は上記の `catkin_ws` を使用する設定を
元に戻しておくことをおすすめします．
ここで追記した2行をファイル `~/.bashrc` 内から削除して下さい．
