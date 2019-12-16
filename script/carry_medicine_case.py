#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from hsrb_task_apu_components import move_to_locations
from hsrb_task_apu_components import return_to_locations
from hsrb_task_apu_components import vacuum

if __name__ == '__main__':
    rospy.init_node('carry_medicine_case')
    rospy.sleep(2)
    mtl = move_to_locations.MoveToLocations()
    trans_time , trans_origin = mtl._move('medicine_calendar')
    if not trans == None:
        rospy.sleep(2)
        vc = vacuum.VacuumCase()
        vc._vacuum()
        rospy.sleep(2)
        rtl = return_to_locations.ReturnToLocations()
        rtl._return_move(trans_time , trans_origin)
