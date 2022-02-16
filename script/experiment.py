#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hsrb_interface import Robot
from hsrb_interface import geometry
import rospy
import math
from std_msgs.msg import Int16

robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')
#rospy.Subscriber('/apu_identify_calendar_node/flag', Int16, flag_call_back)
flag = 0

def flag_call_back(data):
    flag = data.data


if __name__ == '__main__':
    pub = rospy.Publisher('/apu_identify_calendar_node/flag', Int16, queue_size=10)

    d = 0.40
    t = 10
    y_d = d/2
    x_d = y_d*math.sqrt(3)

    try:
        #whole_body.move_to_neutral()
        #base.follow_trajectory([geometry.pose(x=0.0, y=0.0, ek=math.radians(60))], time_from_starts=[3], ref_frame_id='base_footprint')
        #whole_body.move_to_joint_positions({'arm_lift_joint': 0.2,'head_pan_joint':math.radians(-60)})
        rospy.sleep(0.5)
        print('pub flag 1')
        pub.publish(1)
        base.follow_trajectory([geometry.pose(x=-x_d, y=-y_d, ek=0.0)], time_from_starts=[t], ref_frame_id='base_footprint')
        rospy.sleep(0.5)
        pub.publish(2)
        print('pub flag 2')
    except Exception as e:
        rospy.loginfo("Faild {0}".format(e))
