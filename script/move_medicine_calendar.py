#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from hsrb_task_apu_components import move_to_locations
from hsrb_task_apu_components import vacuum
from hsrb_task_apu_components import hsr_say

if __name__ == '__main__':
    rospy.init_node('move_medicine_calendar')
    rospy.sleep(2)
    hs = hsr_say.HsrSay()

    mtl = move_to_locations.MoveToLocations()
    trans_origin = mtl._move('medicine_calendar')
    rospy.sleep(2)
    if not trans_origin == None:
        vc = vacuum.VacuumCase()
        vc._look_medicine_calnedar()
