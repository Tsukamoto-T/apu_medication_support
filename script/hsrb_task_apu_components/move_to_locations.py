#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
import sys
import tf2_ros
import geometry_msgs.msg
from hsrb_interface import geometry
import math
from hsrb_move_to_locations.msg import MoveToLocationsAction
from hsrb_move_to_locations.msg import MoveToLocationsResult
from tmc_navigation_msgs.msg import BaseLocalPlannerStatus

_ACTION_NAME = 'move_to_locations'
_DEFAULT_MOVE_TIMEOUT = 100.0

class MoveToLocations(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._move_timeout = rospy.Duration(_DEFAULT_MOVE_TIMEOUT)
        self._tts = self._robot.get('default_tts')
        self._tf2_buffer = self._robot._get_tf2_buffer()

    def _move(self,frame_ids):
        #look_tf
        listener = tf2_ros.TransformListener(self._tf2_buffer)
        try:
            self._whole_body.move_to_go()
            trans_time = rospy.Time.now()
            trans_origin = self._tf2_buffer.lookup_transform('map','base_footprint',rospy.Time.now(),rospy.Duration(20))
            trans = self._tf2_buffer.lookup_transform('base_footprint',frame_ids,rospy.Time.now(),rospy.Duration(20)) #１つ目の引数のx軸からのtransform
            _yaw = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            self._omni_base.go_rel(0.0, 0.0, _yaw, 300.0)
        except Exception as e:
            rospy.logerr('Fail (transform): {}'.format(e))

        #move
        rospy.sleep(2)
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
                if self._omni_base.is_succeeded():
                    self._tts.say(u'移動しました')
                    return trans_time , trans_origin
            except Exception as e:
                rospy.loginfo("move Faild :{}".format(e))
                continue
        return None
