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
