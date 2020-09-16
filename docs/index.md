# ROSの概要
## 概要
## 歴史
- 2010年、アメリカのWillow Garageが開発した。源流はStanfordのAI Lab（STAIR）で、2018年現在はOSRFが管理している。
- ハードを中心としたロボット指向プログラミングだったのが、ソフトを中心としたモジュール指向プログラミングに変わってきた。
    - ハードに依存しないソフトを開発する必要が出てきた。
- ロボットの開発がソフト中心になってきた。
    - 汎用性、再利用性、移植性などが求められるようになってきた。
- オープンソースの研究開発により、どんどん機能が洗練されていく。
    - ただし、アクティブユーザー数に依存する。
    - 短時間＆ローコストで機能を開発できる。
- 2011年頃はRTMとROSがバチバチしていた。
    - RTMの方がWindowsに対応しているなど、システムとしては優れていたが、ユーザー数はROSの方が多かった。
- ROSの基本は「プロセス間通信のためのライブラリー」と「プログラムのビルドシステム」の2つである。
    - OSとアプリの間にあるミドルウェアの位置付けになる。
    - ソフトから見ると、ドライバーやジョブ管理などのOSの一部として機能しているように見える。
- 2018年現在では、圧倒的にROSが有利な状況となっている。

### 補足
- ROSと同じようなソフトウェアとして、YARP（ヤープ）というロボットを制御するための枠組みもある。イタリアで開発された。
- 2019年4月にNVIDIAがIsaac（アイザック）を提供し始めた。
    - AI研究者がロボットを研究しやすくするためのツールボックス

## 利点
- 最先端のツール群を利用できる。
- 分散型システムを構築できる。
    - 機能を最小単位に分割し、再利用性を高める。
    - バグがシステム全体に影響しないという耐故障性も確保しやすい。
- 複数の言語に対応している。
    - 実際にはC++＆Pythonかな？

## 欠点
- デフォルトの設定では、リアルタイム処理ができない。
- 親（roscore）との通信が切れるとダウンするので危ない。
 - トヨタもシステムダウンを想定してHSRを設計している。
- 画像や点群を高速に遣り取りすることも難しい。

## ROS and others

　|
---|
Player|
YARP|
Orocos|
CARMEN|
Orca|
Microsoft Robotics Studio|


# ROSの環境構築
## Ubuntuの用意
- ROSの演習を行うため、Ubuntu 18.04（Bionic Beaver）が使用できるコンピューターを1人1台用意する。
- 可能であれば、VirtualBoxなどの仮想環境でなく、ネイティブ環境にインストールする。（＝マルチブート環境を構築する。）
- 難しい場合は仮想環境にインストールする。本演習はVirtualBoxでも動作しますが、研究レベルのシミュレーションは動作しません。

## ROSの環境構築
Ubuntu 18.04に対応したROS（Melodic Morenia）をインストールする。  
[ROS.org](http://wiki.ros.org/ROS/Installation)を参考にする。  
ターミナルに下記のコマンドを一行ずつ入力し、実行する。  
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```  
正しくインストールできたかどうかをターミナルで確認する。  
```
$ printenv | grep ROS
↓正常にインストールできていれば、下記のように出力される。
ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ROS_ROOT=/opt/ros/melodic/share/ros
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROS_PYTHON_VERSION=2
ROS_PACKAGE_PATH=/opt/ros/melodic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=melodic
```  
試しにビルドする。  
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ roscd
↓正常にビルドできていれば、下記の場所に移動する。
~/catkin_ws/devel
```  
試しに演習に必要となるROSパッケージをインストールする。  
```
$ sudo apt install ros-melodic-usb-cam
$ sudo apt install ros-melodic-cv-camera
$ sudo apt install ros-melodic-opencv-apps
```



#ROSの動作確認
## turtlesimをキーボードで動かす  
全部で４つのターミナルが必要になります。ターミナルを４つ用意しても良いですし、１つのターミナルに４つのタブを作成しても良いです。

### １つ目のターミナル  
ROSマスターを起動する。  
```
$ roscore
```

### ２つ目のターミナル  
バーチャルなロボットとして、turtlesimノードを起動する。  
```
$ rosrun turtlesim turtlesim_node
```

### ３つ目のターミナル  
亀を動かすためのROSノードを起動する。キーボード入力を速度情報に変換するノードです。  
```
$ rosrun turtlesim turtle_teleop_key
```  
３つ目のターミナルをアクティブにした状態で矢印キーを入力すると、亀を動かせます。

### ４つ目のターミナル  
#### rqt_graph  
ROSノード同士の繋がりを確認します。  
```
$ rosrun rqt_graph rqt_graph（※正式な方法）
又は
$ rqt_graph（※簡便な方法）
```  
名前が「/teleop_turtle」というturtle_teleop_keyタイプのノードと、名前が「/turtlesim」というturtlesim_nodeタイプのノードが、ROSトピック「/turtle1/cmd_vel」で繋がっていることが確認できると思います。このトピックが速度情報を伝達するため、キーボードで亀を動かすことができます。  

閉じるボタンか「Ctrlキー」＋「cキー」でrqt_graphを停止します。

#### rostopic  
下記のコマンドでトピックの値を確認してみましょう。  
```
$ rostopic echo /turtle1/cmd_vel
```  
３つ目のターミナルで矢印キーを押すたびに、４つ目のターミナルに値が表示されると思います。  

重要なポイントは、/teleop_turtleノードは速度情報を配信しているだけ、/turtlesimノードは速度情報を購読しているだけで、どのノードが配信・購読しているかは全く関係ないところです。つまり、速度情報を出すのが、キーボードでも、マウスでも、ジョイスティックでも、画像処理の結果でも、音声処理の結果でも、何でも大丈夫です。同様に、キーボードだけで、バーチャルなロボットも、リアルなロボットも、自動車も、ドローンも、何でも操作することができます。これがROSの利点です！  

「Ctrlキー」＋「cキー」でrostopic echoを終了します。  
もちろん、ターミナルから直接、速度情報を出すこともできます。  
まず、ROSトピック「/turtle1/cmd_vel」の型（メッセージ型）を確認します。  
```
$ rostopic type /turtle1/cmd_vel
```
「geometry_msgs/Twist」という型であることが分かります。もちろんintなどの単純な型（std_msgs/Int32）も存在しますが、ROSはロボット用のソフトウェアなので、いくつかの変数がまとまった構造体のような型を利用することが多いです。  

下記のコマンドで調べると、メッセージ型「geometry_msgs/Twist」は２つの３次元ベクトル（linearとangular）で構成されており、それぞれの値はfloatであることが分かります。  
```
$ rosmsg show geometry_msgs/Twist
```  
そして、次のようなコマンドで６つの値を配信することができます。１つ目の値で前進量、６つ目の値で回転量を指定することができます。ちなみに回転方向は反時計回りが正になります。なお、毎回「Ctrlキー」＋「cキー」で停止する必要があります。  
```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -- '[1.0,0.0,0.0]' '[0.0,0.0,0.0]'
```  
下記のように「-1」オプションを付けることで、１回実行後に自動で終了させることもできます。  
```
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0,0.0,0.0]' '[0.0,0.0,3.14]'
```  
ちなみに、下記のコマンドでトピックの一覧を確認することができます。他のトピックも確認してみてください。  
```
$ rostopic list
```  

#### rosparam  
実はroscoreを実行すると、ROSパラメーターサーバーも一緒に起動します。このサーバーを介して、グローバル変数のように全てのノードで利用可能な変数を共有することができます。  

どんなパラメーターがあるかというと、  
```
$ rosparam list
```  
具体的な値を確認すると、  
```
$ rosparam get /background_b
```  
値を変更すると、  
```
$ rosparam set /background_b 0
```  
変更したパラメーターを再読込すると、  
```
$ rosservice call /clear

```  
亀がいるウィンドウの背景色が変わると思います。  
注意点として、パラメーターはノード起動時に読み込まれるので、起動時に設定するか、あとで再読込（再起動）する必要があります。  

#### 付録  
下記のように実行することで、任意の名前でノードを起動することができます。なお、ROSの仕様により、同じ名前のノードは一つしか存在することができず、同じ名前を指定すると上書きされてしまいます。  
```
$ rosrun turtlesim turtlesim_node __name:=hoge
```  
逆に、異なる名前を指定すれば、複数の亀を発生させることも可能です。

