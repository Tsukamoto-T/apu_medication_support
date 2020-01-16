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

class VacuumCase(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._tts = self._robot.try_get('default_tts')
        self._tf2_buffer = self._robot._get_tf2_buffer()
        self._pub_vacumme = rospy.Publisher('/hsrb/command_suction', Bool, queue_size = 1)
        rospy.Subscriber('/hsrb/pressure_sensor', Bool, self._vacuum_call_back)
        self._find_case = False
        self._result = 0
        self._suction_result = False

    def _vacuum_call_back(self, data):
        self._suction_result = data.data

    def _look_medicine_calnedar(self):
        #look_medicine_calendar
        try:
            self._whole_body.move_to_neutral()
            self._whole_body.move_to_joint_positions({'arm_lift_joint': 0.25})
            self._omni_base.go_rel(0.0,0.0,math.radians(60),200.0)
            self._whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})
            self._whole_body.move_to_joint_positions({'head_pan_joint': math.radians(-58)})
        except Exception as e:
            rospy.loginfo("look_calendar Faild {0}".format(e))

    def _vacuum(self,senser_use):
        #vacuum
        rospy.sleep(2)
        listener = tf2_ros.TransformListener(self._tf2_buffer)
        for num in range(2):
            rospy.sleep(2)
            if (self._tf2_buffer.can_transform('map', 'suction_case_frame',rospy.Time.now(),rospy.Duration(20.0))):
                self._tts.say(u'薬を見つけました')
                self._find_case = True
                try:
                    rospy.sleep(1)
                    self._whole_body.end_effector_frame = u'hand_l_finger_vacuum_frame'
                    self._whole_body.move_end_effector_pose(geometry.pose(x=-0.01,z=-0.03,ek=-1.57),'suction_case_frame')
                    rospy.sleep(1)
                    self._whole_body.impedance_config = 'compliance_middle'
                    self._whole_body.liner_weight=(100)
                    self._whole_body.liner_weight=(100)
                    self._whole_body.move_end_effector_by_line((0,0,1),0.075)
                    self._whole_body.impedance_config = None
                    for num2 in range(15):
                        self._pub_vacumme.publish(True)

                    rospy.sleep(2)

                    if (self._suction_result == True):
                        rospy.loginfo('Suction succeeded')
                        self._result = 1
                        break
                    else:
                        rospy.loginfo('Suction failed')
                        self._result = 2
                        break

                except Exception as e:
                    rospy.loginfo("vacuum Faild {0}".format(e))
                    self._find_case = False
                    self._result = 3
                    break
            else:
                if not (num == 1):
                    self._tts.say(u'薬を検出中です')
                else:
                    self._tts.say(u'薬を見つけられませんでした')
                    self._find_case = False
                    self._result = 0

        rospy.sleep(2)
        if(senser_use == True):
            if not(self._result == 1):
                for num in range(10):
                    self._pub_vacumme.publish(False)
        else:
            if not(self._result == 1 and self._result == 2):
                for num in range(10):
                    self._pub_vacumme.publish(False)

        if (self._find_case == True):
            #return_posture
            self._whole_body.impedance_config = 'compliance_middle'
            self._whole_body.move_end_effector_by_line((1,0,0),0.04)
            self._whole_body.move_end_effector_by_line((0,0,1),-0.1)
            self._whole_body.impedance_config = None
            self._whole_body.move_to_neutral()
            self._whole_body.end_effector_frame = u'hand_palm_link'
        else:
            self._whole_body.move_to_neutral()
