#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
from hsrb_interface import geometry
import math
import sys
import tf2_ros
import geometry_msgs.msg

class HsrSay(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._tts = self._robot.get('default_tts')

    def _say(self, _word):
        self._tts.say(_word)
