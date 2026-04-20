#!/usr/bin/env python
import subprocess
import rospy

NODES_TO_KILL = [
    '/pal_loc_measure',
    '/pal_navigation_sm',
    '/move_base',
    '/pal_vo_server',
    '/map_setup',
]

if __name__ == '__main__':
    rospy.init_node('kill_pal_nav_nodes', anonymous=True)
    rospy.sleep(5)
    for node in NODES_TO_KILL:
        result = subprocess.call(['rosnode', 'kill', node])
        if result == 0:
            rospy.loginfo("Killed %s", node)
        else:
            rospy.logwarn("Could not kill %s (may not be running)", node)
    rospy.loginfo("PAL navigation node cleanup done")
