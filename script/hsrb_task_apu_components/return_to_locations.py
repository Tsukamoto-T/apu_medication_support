#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
import sys
import tf2_ros
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
from hsrb_interface import geometry
from hsrb_move_to_locations.msg import MoveToLocationsAction
from hsrb_move_to_locations.msg import MoveToLocationsResult
from tmc_navigation_msgs.msg import BaseLocalPlannerStatus

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
        listener = tf2_ros.TransformListener(self._tf2_buffer)

    def _return_move(self,trans_time,trans_origin):
        #look_tf
        try:
            self._whole_body.move_to_go()
            trans = self._tf2_buffer.lookup_transform('map','base_footprint',rospy.Time.now(),rospy.Duration(20))
            trans_map = self._tf2_buffer.lookup_transform('base_footprint','map',rospy.Time.now(),rospy.Duration(20))
            _yaw_x = trans_origin.transform.translation.x - trans.transform.translation.x
            _yaw_y = trans_origin.transform.translation.y - trans.transform.translation.y
            _yaw_o = math.atan2(_yaw_y,_yaw_x)
            if (trans_map.transform.rotation.z < 0):
                _yaw = -2*math.acos(trans_map.transform.rotation.w) + _yaw_o
            else:
                _yaw = 2*math.acos(trans_map.transform.rotation.w) + _yaw_o
            self._omni_base.go_rel(0.0, 0.0, _yaw, 300.0)
        except Exception as e:
            rospy.logerr('Fail (transform): {}'.format(e))

        rospy.sleep(2)

        # return_move
        rate = rospy.Rate(10)
        for num in range(3):
            try:
                start_time = rospy.Time.now()
                if (trans_origin.transform.rotation.z < 0):
                    pose_ek = -2*math.acos(trans_origin.transform.rotation.w)
                else:
                    pose_ek = 2*math.acos(trans_origin.transform.rotation.w)
                return_pose = geometry.pose(x=trans_origin.transform.translation.x, y=trans_origin.transform.translation.y, z=trans_origin.transform.translation.z,ek=pose_ek)
                goal = self._omni_base.create_go_pose_goal(return_pose,'map')
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
                    return
            except Exception as e:
                rospy.loginfo("move Faild :{}".format(e))
                continue
        return
