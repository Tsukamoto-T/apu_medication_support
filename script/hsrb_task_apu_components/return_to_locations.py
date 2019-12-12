#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
import sys
import tf2_ros
import geometry_msgs.msg
from hsrb_interface import geometry
import math

_ACTION_NAME = 'move_to_locations'
_DEFAULT_MOVE_TIMEOUT = 100.0

class ReturnToLocations(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._move_timeout = rospy.Duration(_DEFAULT_MOVE_TIMEOUT)
        self._tts = self._robot.get('default_tts')
        self._tf2_buffer = self._robot._get_tf2_buffer()

    def _return_move(self,trans):
        listener = tf2_ros.TransformListener(self._tf2_buffer)
        trans_re = self._tf2_buffer.lookup_transform('base_footprint','map',rospy.Time.now(),rospy.Duration(20))
        try:
            self._whole_body.move_to_go()
            _yaw = math.atan2(trans.transform.translation.y + trans_re.transform.translation.y, trans.transform.translation.x + trans_re.transform.translation.x)
            self._omni_base.go_rel(0.0, 0.0, _yaw, 300.0)
        except Exception as e:
            rospy.logerr('Fail (transform): {}'.format(e))

        rospy.sleep(2)

        frame_ids = 'return_frame'
        rate = rospy.Rate(10)
        for frame in frame_ids:
            try:
                start_time = rospy.Time.now()
                goal = self._omni_base.create_go_pose_goal(geometry.pose(),frame_ids)
                self._omni_base.execute(goal)
                while not rospy.is_shutdown():
                    rate.sleep()
                    if not self._omni_base.is_moving():
                        break
                    if rospy.Time.now() - start_time > self._move_timeout:
                        rospy.logwarn("timeout")
                        break
                if self._omni_base.is_succeed():
                    self._tts.say(u'終了します')
                    return frame
            except Exception as e:
                rospy.loginfo("move Faild :{}".format(e))
                continue
        return None
