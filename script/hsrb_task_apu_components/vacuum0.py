#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
from hsrb_interface import geometry
import math
import sys
import tf2_ros
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)

_CONNECTION_TIMEOUT = 10.0
_SUCTION_TIMEOUT = rospy.Duration(20.0)


class VacuumCase(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._tts = self._robot.try_get('default_tts')
        self._tf2_buffer = self._robot._get_tf2_buffer()

    def _vacuum(self):
        #look_medicine_calendar
        try:
            self._whole_body.move_to_neutral()
            self._whole_body.move_to_joint_positions({'arm_lift_joint': 0.3})
            self._omni_base.go_rel(0.0,0.0,math.radians(60),200.0)
            self._whole_body.move_to_joint_positions({'head_tilt_joint': 0.0})
            self._whole_body.move_to_joint_positions({'head_pan_joint': math.radians(-60)})
        except Exception as e:
            rospy.loginfo("look_calendar Faild {0}".format(e))

        #vacuum
        rospy.sleep(3)
        listener = tf2_ros.TransformListener(self._tf2_buffer)
        for num in range(3):
            if (self._tf2_buffer.can_transform('map', 'suction_case_frame',rospy.Time.now(),rospy.Duration(20.0))):
                self._tts.say(u'薬を見つけました')
                try:
                    rospy.sleep(1)
                    self._whole_body.end_effector_frame = u'hand_l_finger_vacuum_frame'
                    self._whole_body.move_end_effector_pose(geometry.pose(x=-0.01,z=-0.03,ek=-1.57),'suction_case_frame')
                    rospy.sleep(1)
                    self._whole_body.impedance_config = 'compliance_soft'
                    self._whole_body.move_end_effector_by_line((0,0,1),0.05)
                    self._whole_body.impedance_config = None
                    # Create action client to control suction
                    suction_action = '/hsrb/suction_control'
                    suction_control_client = actionlib.SimpleActionClient(suction_action, SuctionControlAction)
                    # Wait for connection
                    try:
                        if not suction_control_client.wait_for_server(rospy.Duration(_CONNECTION_TIMEOUT)):
                            raise Exception(suction_action + ' does not exist')
                    except Exception as e:
                        rospy.logerr(e)
                        sys.exit(1)

                    # Send a goal to start suction
                    rospy.loginfo('Suction will start')
                    suction_on_goal = SuctionControlGoal()
                    suction_on_goal.timeout = _SUCTION_TIMEOUT
                    suction_on_goal.suction_on.data = True
                    if (suction_control_client.send_goal_and_wait(suction_on_goal) ==GoalStatus.SUCCEEDED):
                        rospy.loginfo('Suction succeeded. Suction will stop')
                        # Send a goal to stop suction
                        suction_off_goal = SuctionControlGoal()
                        suction_off_goal.suction_on.data = True
                        suction_control_client.send_goal_and_wait(suction_off_goal)
                        break
                    else:
                        rospy.loginfo('Suction failed')
                        break

                except Exception as e:
                    rospy.loginfo("vacuum Faild {0}".format(e))
                    break
            else:
                if (num==0 or num==1):
                    self._tts.say(u'薬を検出中です')
                else:
                    self._tts.say(u'薬を見つけられませんでした')

        #return_posture
        rospy.sleep(2)
        self._whole_body.impedance_config = 'compliance_soft'
        self._whole_body.move_end_effector_by_line((1,0,0),0.04)
        self._whole_body.move_end_effector_by_line((0,0,1),-0.1)
        self._whole_body.impedance_config = None
        self._whole_body.move_to_neutral()
        self._whole_body.end_effector_frame = u'hand_palm_link'
