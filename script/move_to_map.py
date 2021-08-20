#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from hsrb_task_apu_components import move_to_locations

if __name__ == '__main__':
    rospy.init_node('move_to_map')
    rospy.sleep(2)
    mtl = move_to_locations.MoveToLocations()
    trans_origin = mtl._move('map')
    if not trans_origin == Non:
        mtl._tts.say(u'移動しました')
    else:
        mtl._tts.say(u'移動に失敗しました')
