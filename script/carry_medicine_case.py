#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from hsrb_task_apu_components import move_to_locations
#from hsrb_task_apu_components import return_to_locations
from hsrb_task_apu_components import vacuum
from hsrb_task_apu_components import hand_over_case
from hsrb_task_apu_components import hsr_say

if __name__ == '__main__':
    rospy.init_node('carry_medicine_case')
    rospy.sleep(2)
    hs = hsr_say.HsrSay(u'薬を持ってきます')

    mtl = move_to_locations.MoveToLocations()
    for num in range(3):
        trans_origin = mtl._move('medicine_calendar')
        rospy.sleep(2)
        if not trans_origin == None:
            vc = vacuum.VacuumCase()
            vc._look_medicine_calnedar()
            vc._vacuum()
            rospy.sleep(2)

            if (vc._result == 0):
                trans_ = mtl._move('living_room')
                rospy.sleep(2)
                hs._say(u'今、のむ薬はありません')
                break

            elif (vc._result == 1):
                trans_ = mtl._move('living_room')
                rospy.sleep(2)
                hoc = hand_over_case.HandOverCase()
                hoc._hand_over()
                rospy.sleep(2)
                hs._say(u'薬を持ってきました')
                break

            elif (vc._result == 3):
                hs._say(u'エラーが発生しました')
                break

            else:
                hs._say(u'薬ケースの吸引に失敗しました。もう一度行います')

        else:
            hs._say(u'移動できませんでした')
            break
    '''
        rtl = return_to_locations.ReturnToLocations()
        rtl._return_move(trans_origin)
    '''
