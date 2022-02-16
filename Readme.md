# apu_medication_support

# Author
tsukamoto tomoki

# Date
2021/08/20

# 構成
## launch
- identify_case：薬の認識のみ実行
- carry_case(_rsh)：薬の認識、運搬

- ex_transparent_detection：実験１
- ex_2_transparent_detection：実験２
- transparent_detection：透明物体認識（不完全）

## script
- hsrb_task_apu_components：基本的な動作（吸引、移動、発話など）の関数
- carry_case：薬ケース運搬動作
- move_medicine_calendar：薬カレンダー前に移動
- experiment：実験用

## src
- identify_case：薬ケース認識
- edge_detection：エッジ検出
- transparent_detection：透明検出
- ex_：実験用
- ex_plot：データ分析


## memo
 - rosbag record /tf /tf_static /hsrb/head_rgbd_sensor/depth_registered/camera_info /hsrb/head_rgbd_sensor/depth_registered/rectified_points /hsrb/head_rgbd_sensor/rgb/camera_info /hsrb/head_rgbd_sensor/rgb/image_rect_color /hsrb/head_rgbd_sensor/depth_registered/image_rect_raw -O calendar.bag：bag取得

- rostopic pub /apu_identify_calendar_node/flag std_msgs/Int16 0：flag投げる

- rosparam set /use_sim_time true
- rosbag play --clock -q src/apu_medication_support/bag/calendar_mv.bag：bag実行

- omni_base.follow_trajectory([geometry.pose(x=0.0,y=1.0,ek=0.0)],time_from_starts=[40],ref_frame_id='base_footprint')：横移動

- sudo update-alternatives --config python：python切り替え
