#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
from hsrb_interface import geometry
import math
import sys
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool

class HandOverCase(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._tts = self._robot.try_get('default_tts')
        self._pub_vacumme = rospy.Publisher('/hsrb/command_suction', Bool, queue_size = 1)

    def _hand_over(self):
        try:
            self._whole_body.move_to_neutral()
            self._whole_body.looking_hand_constraintraint = True
            self._whole_body.move_to_joint_positions({'arm_flex_joint':-0.7,'wrist_flex_joint':-0.9})
            self._whole_body.end_effector_frame = u'hand_l_finger_vacuum_frame'
            self._whole_body.angular_weight=(100)
            self._whole_body.move_end_effector_pose(geometry.pose(),'hand_palm_link')
            for num2 in range(10):
                self._pub_vacumme.publish(False)
            self._tts.say(u'薬を持ってきました')
        except Exception as e:
            rospy.loginfo("hand_over_case Faild {0}".format(e))
