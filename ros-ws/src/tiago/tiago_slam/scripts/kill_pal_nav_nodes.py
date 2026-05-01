#!/usr/bin/env python
import subprocess
import time
import rospy

NODES_TO_KILL = [
    '/pal_loc_measure',
    '/pal_navigation_sm',
    '/pal_vo_server',
    '/map_setup',
    '/amcl',
    '/map_server',
    '/compressed_map_publisher',
    '/map_aligner_node',
    '/map_configuration_server',
    '/move_base',
    '/move_base_with_arm_demo',
    '/poi_navigation_server',
    '/robot_pose_publisher',
]

RETRIES = 1

if __name__ == '__main__':
    rospy.init_node('kill_pal_nav_nodes', anonymous=True)
    rospy.sleep(5)
    for node in NODES_TO_KILL:
        killed = False
        for attempt in range(RETRIES):
            result = subprocess.call(['rosnode', 'kill', node])
            if result == 0:
                rospy.loginfo("Killed %s", node)
                killed = True
                break
        if not killed:
            rospy.logwarn("Could not kill %s (may not be running)", node)
    rospy.loginfo("PAL navigation node cleanup done")
