#!/usr/bin/env python
import rospy

def node():
    """
    Dummy node to simulate the scanning procedure, i.e. to scan on command.
    """
    rospy.init_node('dummy_scanner')

    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
