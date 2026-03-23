#!/usr/bin/env python
import subprocess
import rospy

if __name__ == '__main__':
    rospy.init_node('kill_head_nodes', anonymous=True)
    rospy.sleep(5)
    subprocess.call(['rosnode', 'kill', '/pal_head_manager'])
    subprocess.call(['rosnode', 'kill', '/look_to_cmd_vel/look_at_cmd_vel_node'])
    rospy.loginfo("Head movement nodes killed")
