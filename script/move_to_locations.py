import hsrb_interface
import rospy
from hsrb_interface import geometry

_ACTION_NAME = 'move_to_locations'
_DEFAULT_MOVE_TIMEOUT = 100.0

class MoveToLocations(object):
    def __init__(self):
        self._robot = hsrb_interface.Robot()
        self._omni_base = self._robot.get('omni_base')
        self._whole_body = self._robot.get('whole_body')
        self._move_timeout = rospy.Duration(_DEFAULT_MOVE_TIMEOUT)

    def _move(self,frame_ids):
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
                    return frame
            except Exception as e:
                rospy.loginfo("move Faild {0}".format(e))
                continue
        return None

if __name__ == '__main__':
    rospy.sleep(1)
    mtl=MoveToLocations()
    mtl._whole_body.move_to_go()
    rospy.sleep(1)
    arrival = mtl._move('map')
